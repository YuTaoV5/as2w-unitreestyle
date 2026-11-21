set -eu
cd /root/as2w
d=$1
run() {
 local name=$1 pd=$2 case=$3
 local a=unitree-flat-v2-dr/assets/as2w-t2-v2-mu025.usd
 local flags=(--usd)
 if [[ "$case" == native ]]; then a=calibration/candidates/t2-v2-20261119/as2w-t2-v2.urdf; flags=(--native); fi
 if [[ "$case" == max ]]; then export STAND_MU=.5 STAND_COMBINE=max; else unset STAND_MU STAND_COMBINE; fi
 cp "him_arc/physx-$name-fixture.json" "$d/$name-fixture.json"
 ./preflight/isaac-python him_arc/static_contact_holdout_material.py "${flags[@]}" --asset "$a" --fixture "$d/$name-fixture.json" --pd "$pd" --out "$d/$name-$case.json" >"$d/$name-$case.log" 2>&1
 grep -q STATIC_ACCEPTANCE_OK "$d/$name-$case.log"
}
for case in native min max; do
 run t2-repeat1 55 "$case"
 run t2-repeat3 55 "$case"
 run official-repeat 200 "$case"
done
echo STATIC_MATRIX_DONE
