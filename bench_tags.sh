source install/setup.bash
CFG=src/nontok_localization/config/detector.yaml
bench() {   # bench <label> <rate> <cams...>
  label=$1 rate=$2; shift 2
  for c in "$@"; do
    timeout 45 ros2 run nontok_localization tag_detector --ros-args -r __node:=td_$c --params-file $CFG \
      -p camera_name:=$c -p device:=/dev/cam_$c -p rate_hz:=$rate >/tmp/td_$c.log 2>&1 &
  done
  sleep 10; hz=""
  for c in "$@"; do (timeout 15 ros2 topic hz /tags/$c/detections 2>&1 | grep "average rate" | tail -1 | sed "s|^|$label $c: |") & hz="$hz $!"; done
  sleep 4
  top -b -n 2 -d 5 | awk '/^%Cpu/{c=$0} /^ *[0-9]+ /{if($12 ~ /tag_detec|python3/) p[$1]=$9} END{print "  " c; for(k in p) print "  pid",k,p[k]"% of one core"}' | tail -4
  wait $hz; pkill -f tag_detector; sleep 3
}
bench "A both@6 " 6.0 front_left front_right
bench "B both@30" 30.0 front_left front_right
bench "C left@30" 30.0 front_left
