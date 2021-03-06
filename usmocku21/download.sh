#!/bin/bash

## #{ download_data()
download_data() {
  URL=$1
  DESTINATION_FOLDER=$2
  if wget --no-check-certificate --content-disposition "${URL}" -c -P "${DESTINATION_FOLDER}"; then
    echo "wget of data was successfull"
  else
    echo "wget of data was unsuccessfull"
    echo " url: $URL"
    echo " destination: $DESTINATION_FOLDER"
  fi
} 
## #}

######################################################
# Add datasets here:
echo Downloading data for dataset: cisar21
URL_FOLLOWING=TBD
URL_CATCH=TBD

# URLs for files and their respective subdirectories
URL=( "$URL_FOLLOWING" "$URL_CATCH" )
DATA_FOLDERS=( "following" "catch" )
######################################################

# Do not change below!
SCRIPT_PATH="$( cd "$(dirname "$0")" ; pwd -P )"
for (( j=0; j<${#URL[@]}; j++ ));
do
  download_data "${URL[$j]}" "$SCRIPT_PATH/${DATA_FOLDERS[$j]}"
done
