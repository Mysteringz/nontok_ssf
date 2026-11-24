from nontok_viz.telemetry import Telemetry, system_stats, thin_path, yaw_of


def test_snapshot_reports_values_ages_and_rates():
    now = [100.0]
    t = Telemetry(clock=lambda: now[0])
    for k in range(51):                       # 10 messages a second for 5 s
        now[0] = 100.0 + k * 0.1
        t.set('pose', [k, 0, 0])
    t.set('mission', {'state': 'S0_DOCKED'})
    now[0] += 3.0
    snap = t.snapshot()
    assert snap['values']['pose'] == [50, 0, 0] and snap['values']['mission']['state'] == 'S0_DOCKED'
    assert abs(snap['age']['pose'] - 3.0) < 0.01          # gone quiet: the page shows it as stale
    assert 9.0 <= snap['rate']['pose'] <= 11.0


def test_log_is_bounded_and_events_are_kept_in_order():
    t = Telemetry(log_length=3)
    for k in range(5):
        t.log('WARN', 'node', 'message %d' % k)
    t.event('wake word')
    snap = t.snapshot()
    assert [entry['text'] for entry in snap['log']] == ['message 2', 'message 3', 'message 4']
    assert snap['events'][-1]['text'] == 'wake word'


def test_helpers():
    assert abs(yaw_of(0.7071068, 0.7071068) - 1.5707963) < 1e-5
    path = thin_path([(i * 0.01, 0.0) for i in range(1000)], 50)
    assert len(path) == 50 and path[0] == [0.0, 0.0] and path[-1] == [9.99, 0.0]
    files = {'/proc/loadavg': '1.50 1.0 0.5 1/100 1', '/proc/meminfo': 'MemTotal: 4096000 kB\nMemAvailable: 2048000 kB\n',
             '/sys/devices/virtual/thermal/thermal_zone0/temp': '41500\n'}

    def read(path):
        if path not in files:
            raise OSError(path)
        return files[path]
    assert system_stats(read) == {'load1': 1.5, 'mem_used_mb': 2000, 'mem_total_mb': 4000, 'temp_c': 41.5}
