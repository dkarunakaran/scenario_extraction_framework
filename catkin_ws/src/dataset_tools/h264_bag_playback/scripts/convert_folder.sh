
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

input_folder=$1

#find $input_folder/* -prune -type d | while IFS= read -r d; do
d=${input_folder}
  for input_file in $d/*[0-9].h264; do
    if [ ! -f "$input_file" ]; then
      echo "Could not find video files in $d"
    else
      output_file=$(echo $input_file | sed "s/.h264/.mp4/g")
      temp_file=$(echo $input_file | sed "s/.h264/.tmp.mp4/g")
      echo "Found h264 file $input_file"
      if [ ! -f "$output_file" ]; then
        echo -e "${GREEN}Writing to $output_file${NC}"
        ffmpeg -nostats -hide_banner -loglevel error -y -i $input_file -map 0:v -vcodec copy -bsf:v h264_mp4toannexb $temp_file
        ffmpeg -nostats -hide_banner -loglevel error -y -fflags +genpts -r 30 -i $temp_file -vcodec copy $output_file
        rm $temp_file
      else
        echo -e "${YELLOW}$output_file already exists${NC}"
      fi
    fi
  done
#done

