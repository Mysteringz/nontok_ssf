"""Tray and classifier logic on synthetic images. No camera, no model."""
import pathlib

import cv2
import numpy as np
import pytest

from nontok_perception import tray_logic
from nontok_perception.classifier import load_items, preprocess, probabilities, vote
from nontok_perception.tray_logic import TrayConfig, TrayLogic

W, H = 640, 480


def empty_tray(brightness=150, seed=0):
    rng = np.random.default_rng(seed)
    frame = np.full((H, W, 3), brightness, dtype=np.float32)
    frame += rng.normal(0, 2.0, frame.shape)           # sensor noise
    return np.clip(frame, 0, 255).astype(np.uint8)


def with_item(frame, centre, size=(60, 30), colour=(30, 40, 200)):
    out = frame.copy()
    cx, cy = centre
    cv2.rectangle(out, (cx - size[0] // 2, cy - size[1] // 2), (cx + size[0] // 2, cy + size[1] // 2), colour, -1)
    return out


def ready_logic():
    logic = TrayLogic(TrayConfig())
    logic.set_reference([empty_tray(seed=i) for i in range(5)])
    return logic


def test_no_reference_reports_nothing():
    reading = TrayLogic().update(empty_tray(), 0.0)
    assert reading.state == tray_logic.EMPTY and not reading.has_reference


def test_empty_stays_empty_despite_noise_and_brightness_change():
    logic = ready_logic()
    for i in range(20):
        reading = logic.update(empty_tray(seed=100 + i), i * 0.2)
        assert reading.state == tray_logic.EMPTY and reading.occupied_fraction < 0.005
    # The whole image gets brighter (auto-exposure): still empty once it has settled.
    logic.update(empty_tray(brightness=185, seed=1), 5.0)
    reading = logic.update(empty_tray(brightness=185, seed=2), 5.2)
    assert reading.state == tray_logic.EMPTY


def test_item_placed_then_stable_then_removed():
    logic = ready_logic()
    t = 0.0
    for i in range(5):
        logic.update(empty_tray(seed=10 + i), t)
        t += 0.2
    # A hand sweeps through: big changes between frames.
    reading = logic.update(with_item(empty_tray(seed=20), (200, 200), size=(260, 160), colour=(60, 90, 140)), t)
    assert reading.state == tray_logic.MOVING and not reading.stable
    t += 0.2
    # The hand leaves; a small item (about 3 % of the tray) stays.
    states = []
    for i in range(14):
        reading = logic.update(with_item(empty_tray(seed=30 + i), (320, 240), size=(110, 60)), t)
        states.append((reading.state, reading.stable))
        t += 0.2
    assert states[0][0] == tray_logic.MOVING            # the hand disappearing is motion too
    assert states[2] == (tray_logic.OCCUPIED, False)    # there, but not yet still for 2 s
    assert states[-1] == (tray_logic.OCCUPIED, True)
    # Removed.
    logic.update(empty_tray(seed=60), t)
    reading = logic.update(empty_tray(seed=61), t + 0.2)
    assert reading.state == tray_logic.EMPTY


def test_small_item_is_detected():
    """An item covering about 2.5 % of the tray floor must register."""
    logic = ready_logic()
    frame = with_item(empty_tray(seed=5), (300, 260), size=(90, 55))
    logic.update(frame, 0.0)
    reading = logic.update(with_item(empty_tray(seed=6), (300, 260), size=(90, 55)), 0.2)
    assert reading.state == tray_logic.OCCUPIED


def test_hand_outside_the_tray_region_is_ignored():
    logic = ready_logic()
    frame = with_item(empty_tray(seed=7), (30, 240), size=(50, 400))     # left of the tray region
    logic.update(frame, 0.0)
    assert logic.update(frame, 0.2).state == tray_logic.EMPTY


def test_item_crop_is_centred_square_and_scaled():
    logic = ready_logic()
    frame = with_item(empty_tray(seed=8), (400, 200), size=(80, 40), colour=(0, 0, 255))
    crop = logic.item_crop(frame)
    assert crop.shape == (224, 224, 3)
    red = (crop[:, :, 2] > 200) & (crop[:, :, 0] < 60)
    ys, xs = np.nonzero(red)
    assert abs(xs.mean() - 112) < 6 and abs(ys.mean() - 112) < 6          # centred
    assert 0.6 < (xs.max() - xs.min()) / 224 < 0.8                        # 80 px in a 112 px box
    assert logic.item_crop(empty_tray(seed=9)) is None


def test_item_crop_at_the_edge_and_minimum_size():
    logic = ready_logic()
    crop = logic.item_crop(with_item(empty_tray(seed=10), (566, 420), size=(16, 16), colour=(0, 0, 255)))
    assert crop.shape == (224, 224, 3)
    red = (crop[:, :, 2] > 200) & (crop[:, :, 0] < 60)
    ys, xs = np.nonzero(red)
    assert abs(xs.mean() - 112) < 8 and abs(ys.mean() - 112) < 8          # still centred, by padding
    assert (xs.max() - xs.min()) < 224 * 0.25                             # 16 px in the 96 px minimum box


def test_preprocess_layout():
    crop = np.zeros((224, 224, 3), dtype=np.uint8)
    crop[:, :, 2] = 255                                   # pure red in BGR
    tensor = preprocess(crop)
    assert tensor.shape == (1, 3, 224, 224) and tensor.dtype == np.float32
    assert tensor[0, 0].min() == 1.0 and tensor[0, 1].max() == 0.0 and tensor[0, 2].max() == 0.0
    assert preprocess(np.zeros((100, 100, 3), dtype=np.uint8)).shape == (1, 3, 224, 224)


def test_probabilities_accepts_both_export_styles():
    assert probabilities(np.array([[0.1, 0.7, 0.2]])) == pytest.approx([0.1, 0.7, 0.2])
    softmaxed = probabilities(np.array([[1.0, 3.0, 2.0]]))
    assert softmaxed.sum() == pytest.approx(1.0) and int(np.argmax(softmaxed)) == 1


def test_vote():
    assert vote([('bolt', 0.9)] * 5) == (True, 'bolt', pytest.approx(0.9))
    assert vote([('bolt', 0.9)] * 3 + [('nut', 0.9)] * 2)[:2] == (True, 'bolt')
    assert vote([('bolt', 0.95), ('bolt', 0.6), ('bolt', 0.6)])[0] is False     # low mean confidence
    assert vote([('bolt', 0.99), ('nut', 0.99)])[0] is False                    # no majority
    assert vote([]) == (False, '', 0.0)


def test_items_file_maps_to_known_stations():
    items = load_items(str(pathlib.Path(__file__).parents[1] / 'config' / 'items.yaml'))
    assert items and set(items.values()) == {'electrical', 'mechanical', 'tools'}


def test_clip_preprocess_gives_a_normalised_square_from_any_shape():
    from nontok_perception.classifier import CLIP_MEAN, CLIP_STD, clip_preprocess
    image = np.full((120, 300, 3), 255, dtype=np.uint8)
    image[:, :, 0] = 0                                     # BGR: no blue, full green and red
    out = clip_preprocess(image)
    assert out.shape == (1, 3, 224, 224) and out.dtype == np.float32
    expected = (np.array([1.0, 1.0, 0.0]) - CLIP_MEAN) / CLIP_STD   # channels come out as RGB
    assert np.allclose(out[0, :, 100, 100], expected, atol=1e-4)


def test_zero_shot_picks_the_closest_description_and_reports_non_items_as_nothing(tmp_path):
    from nontok_perception.classifier import load_descriptions, zero_shot_scores
    text = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    scores = zero_shot_scores(np.array([0.1, 2.0, 0.2]), text)
    assert int(np.argmax(scores)) == 1 and abs(scores.sum() - 1.0) < 1e-9 and scores[1] > 0.99
    path = tmp_path / 'items.yaml'
    path.write_text('items:\n  bolt: mechanical\n  jumper_wire: electrical\n'
                    'descriptions:\n  bolt: [a bolt, a metal screw]\nnot_items:\n  empty: an empty tray\n')
    assert load_descriptions(str(path)) == {'bolt': ['a bolt', 'a metal screw'], 'jumper_wire': ['jumper wire'],
                                            '_empty': ['an empty tray']}


def test_look_alikes_at_one_station_add_up():
    from nontok_perception.classifier import pick_by_station
    stations = {'bolt': 'mechanical', 'nut': 'mechanical', 'resistor': 'electrical'}
    labels = ['bolt', 'nut', 'resistor', '_empty']
    item, confidence = pick_by_station(labels, [0.30, 0.35, 0.33, 0.02], stations)
    assert item == 'nut' and abs(confidence - 0.65) < 1e-9          # resistor had the single highest rival score
    assert pick_by_station(labels, [0.1, 0.1, 0.1, 0.7], stations) == ('', 0.7)   # an empty tray is no item
