from nontok_voice.wake_logic import WakeTrigger

STEP = 0.08   # one audio block


def feed(trigger, scores, t0=0.0):
    return [trigger.update(score, t0 + i * STEP) for i, score in enumerate(scores)]


def test_needs_two_of_three_blocks():
    assert feed(WakeTrigger(), [0.1, 0.9, 0.1, 0.1, 0.1]) == [False] * 5        # a single spike
    assert feed(WakeTrigger(), [0.1, 0.9, 0.9]) == [False, False, True]
    assert feed(WakeTrigger(), [0.9, 0.2, 0.9]) == [False, False, True]         # 2 of the last 3


def test_one_call_gives_one_trigger():
    trigger = WakeTrigger()
    fired = feed(trigger, [0.9] * 20)                 # the word stays "hot" for 1.6 s
    assert sum(fired) == 1
    assert trigger.update(0.9, 5.0) is False          # refractory time over: counting starts fresh
    assert trigger.update(0.9, 5.08) is True


def test_threshold_and_reset():
    assert not any(feed(WakeTrigger(threshold=0.7), [0.6] * 10))
    trigger = WakeTrigger()
    trigger.update(0.9, 0.0)
    trigger.reset()                                   # listening was switched off and on again
    assert trigger.update(0.9, 0.1) is False


def test_microphone_is_found_by_part_of_its_name_even_when_two_share_it():
    from nontok_voice.wake_logic import find_microphone
    devices = [{'name': 'HDMI 0', 'max_input_channels': 0}, {'name': '1080P USB Camera: Audio', 'max_input_channels': 1},
               {'name': 'XWF 1080P  PC Camera: USB Audio (hw:4,0)', 'max_input_channels': 1},
               {'name': 'XWF 1080P  PC Camera: USB Audio (hw:5,0)', 'max_input_channels': 1}]
    assert find_microphone(devices, 'xwf') == 2
    assert find_microphone(devices, '') is None and find_microphone(devices, '3') == 3


def test_the_robot_speaks_at_the_right_moments(tmp_path):
    from nontok_voice.speech_logic import all_phrases, sound_for_display, sound_for_transition
    assert sound_for_transition('S0_DOCKED', 'S1_UNDOCK') == 'answer'
    assert sound_for_transition('S4_CLASSIFY', 'S4_LEAVE_PICKUP', 'bolt') == 'item_bolt'
    assert sound_for_transition('S7_LEAVE_STATION', 'S8_TO_DOCK') == 'going_home'
    assert sound_for_transition('S3_WAIT_ITEM', 'S8_TO_DOCK') == 'no_item'
    assert sound_for_transition('S9_DOCK', 'S0_DOCKED') == 'docked'
    assert sound_for_transition('S5_TO_STATION', 'FAULT', 'STATION_UNREACHABLE') == 'fault_STATION_UNREACHABLE'
    assert sound_for_transition(None, 'S0_DOCKED') is None                 # start-up says nothing
    assert sound_for_transition('S5_TO_STATION', 'S5_TO_STATION') is None  # only on a change
    assert sound_for_display('PLACE ITEM') == 'place_item' and sound_for_display('TO TOOLS') is None
    path = tmp_path / 'speech.yaml'
    path.write_text('phrases:\n  answer: "Yes?"\n  item: "That is {item}, for {station}."\n'
                    'spoken_items:\n  pliers: a pair of pliers\n')
    assert all_phrases(str(path), {'pliers': 'tools', 'offcut': 'scraps'}) == {
        'answer': 'Yes?', 'item_pliers': 'That is a pair of pliers, for tools.', 'item_offcut': 'That is an offcut, for scraps.'}
