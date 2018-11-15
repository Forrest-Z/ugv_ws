
#!/bin/sh
exit_key=Q
base_key=1
navi_key=2
isnavirun=0
isbaserun=0

gnome-terminal -x bash  "roscore; exec $SHELL"

source_file="$(dirname "$(realpath $0)")/devel/setup.bash"
echo "Source file is set AT $source_file";echo

echo "Press Number $base_key to Run Basic Functions Node"
echo "Press Number $navi_key to Run Navigation Node"
echo "Press SHIFT +  q to Terminated the Program"


while [[ 0 ]]
do
	read -s -n1  key

	if [[ $key == $base_key ]]
		then
		if [[ $isbaserun == 0 ]]
		then
		echo
		echo "Starting Base and Sensor Node"
		isbaserun=1
		sleep 2
		gnome-terminal -x bash -c "roscore; exec $SHELL"
		sleep 1
		gnome-terminal -x bash -c "source $source_file; roslaunch param base.launch; exec $SHELL"
		sleep 1
		gnome-terminal -x bash -c "source $source_file; roslaunch param sensor.launch; exec $SHELL"
		echo
		echo "Base and Sensor Node are Running"	
		echo "Press Number $navi_key to Run Navigation Node"
		else
		echo
		echo "Base and Sensor Node have Already Run"
		fi

	elif [[ $key == $navi_key ]] 
	then
		if [[ $isbaserun == 0 ]]
		then
		echo
		echo "You Need Run Basic Functions Node First"
		echo "Press Number $base_key to Run Basic Functions Node"
		elif [[ $isnavirun == 0 ]]
		then
		echo
		echo "Starting Navigation Node"
		isnavirun=1
		sleep 2
		gnome-terminal -x bash -c "source $source_file; roslaunch param navi.launch; exec $SHELL"
		echo "Navigation Node is Running"
		else
		echo
		echo "Navigation Node has Already Run"
		fi
	elif [[ $key == $exit_key ]]
	then
		echo
		echo "Terminating Program.."
		echo "Terminating Program...."
		echo "Terminating Program......"
		sleep 2
		killall gnome-terminal-server
		break
	else
		echo
		echo "You are Pressing Key $key" ;echo
		echo "Press Number $base_key to Run Basic Functions Node"
		echo "Press Number $navi_key to Run Navigation Node"
		echo "Press SHIFT +  q to Terminated the Program"
	fi

done
