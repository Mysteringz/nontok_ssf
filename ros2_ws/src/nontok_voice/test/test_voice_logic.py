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
