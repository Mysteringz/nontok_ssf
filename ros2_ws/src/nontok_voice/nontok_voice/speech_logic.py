"""Which phrase the robot says at which moment. Pure Python; the phrases are in phrases.yaml."""
from typing import Dict, Optional

import yaml


def sound_for_transition(previous: Optional[str], state: str, detail: str = '') -> Optional[str]:
    """Name of the sound to play when the mission goes from `previous` to `state`, or None."""
    if previous is None or previous == state:
        return None
    if state == 'S1_UNDOCK' and previous == 'S0_DOCKED':
        return 'answer'
    if state == 'S4_LEAVE_PICKUP':
        return 'item_' + detail if detail else None
    if state == 'S7_DUMP':
        return 'delivered'
    if state == 'S8_TO_DOCK':
        # Straight from waiting or the pickup drive: nothing was carried.
        return 'going_home' if previous == 'S7_LEAVE_STATION' else 'no_item'
    if state == 'S0_DOCKED' and previous == 'S9_DOCK':
        return 'docked'
    if state == 'FAULT':
        return 'fault_' + detail if detail else 'fault'
    return None


def sound_for_display(text: str) -> Optional[str]:
    """The mission's prompts on the display that are also worth saying."""
    return {'PLACE ITEM': 'place_item', 'REPOSITION ITEM': 'reposition'}.get(text.strip())


def spoken_item(item: str, spoken: Dict[str, str]) -> str:
    if item in spoken:
        return spoken[item]
    words = item.replace('_', ' ')
    return ('an ' if words[:1] in 'aeiou' else 'a ') + words


def all_phrases(speech_path: str, items: Dict[str, str]) -> Dict[str, str]:
    """Sound name -> sentence, with one 'item_<name>' entry per item in items.yaml."""
    with open(speech_path) as handle:
        data = yaml.safe_load(handle) or {}
    phrases, spoken = data.get('phrases') or {}, data.get('spoken_items') or {}
    out = {name: str(text) for name, text in phrases.items() if name != 'item'}
    for item, station in items.items():
        out['item_' + item] = str(phrases['item']).format(item=spoken_item(item, spoken), station=station)
    return out
