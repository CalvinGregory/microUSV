trap "exit" INT TERM ERR
trap "kill 0" EXIT

./CVSensorSimulator/webcam_settings.sh &

./CVSensorSimulator/CVSensorSimulator CVSensorSimulator/config.json &

parallel-ssh -i -h hosts_musv python microUSV/MUSVController/src/mUSV/MUSVClient.py microUSV/MUSVController/src/mUSV/config.json &

wait
