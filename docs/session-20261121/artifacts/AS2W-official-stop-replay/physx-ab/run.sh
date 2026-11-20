set -eu
cd /root/as2w
d=$1
a=calibration/candidates/t2-v2-20261119/as2w-t2-v2.urdf
b=unitree-flat-v2-dr/assets/as2w-t2-v2-mu025.usd
sha256sum "$a" "$b" > "$d/assets.sha256"
for x in A A1 B; do
 case "$x" in
 A) args=(--asset "$a" --native);;
 A1) args=(--asset "$a" --native --base-mass-delta .5 --com-delta-x .009161572);;
 B) args=(--asset "$b" --usd);;
 esac
 REPLAY_HEIGHT=.37 ./preflight/isaac-python him_arc/official_stop_contact_ab.py "${args[@]}" --fixture him_arc/official_stop_ab_input.npz --out "$d/$x.json" >"$d/$x.log" 2>&1
 grep -q REPLAY_OK "$d/$x.log"
done
