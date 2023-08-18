ROOT="../.."
MAP_PATH="example_problems/random.domain/maps/random-32-32-20.map"
PLAN_PATH="test.json"

cd viz/script

python3 plan_viz.py --map $ROOT/$MAP_PATH --plan $ROOT/$PLAN_PATH --grid --aid --tid --static --ca