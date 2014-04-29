#!/bin/sh
options=(/dev/tty.usbserial-????????B)
if  [ ${#options[@]} == 1 ]
then
  if [ -e "${options[0]}" ]
  then
      echo "${options[0]}"
  else
      options=(/dev/tty.usbmodem*)
      if  [ ${#options[@]} == 1 ]
      then
	  if [ -e "${options[0]}" ]
	  then
	      echo "${options[0]}"
	  else
	      echo "no device found"
	  fi
      else
	  select opt in "${options[@]}";
	  do
	    echo "$opt"
	    break;
	  done
      fi
  fi
else
	select opt in "${options[@]}";
	do
		echo "$opt"
		break;
	done
fi
