########################################
# Arg 1 : Targets
# Arg 2 : Trackers
#
# To engage dynamics use -d, --dynamics
########################################
DYNAMICS=false
BUILD=false
for i in "$@"
do
	case $i in
		-b|--build)
			BUILD=true
		;;
	esac
done

python generate_sim.py $1 $2

if $BUILD
then
	catkin build
	source ../../../devel/setup.bash
fi

roslaunch resl_coverage resl_simulation_spawn.launch
