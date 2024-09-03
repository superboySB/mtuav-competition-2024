#!/bin/bash

# [IMPORTANT] put your ak and sk here
AK="3b0859ed3c9d2fd4d7f2a618b85ca413"
SK="4917d1b42d7abc01a5542688c793ff4c"

# server address, DO NOT change unless notified
HOST1="http://sim.race.meituan.com:8090"
HOST2="http://sim.race.meituan.com:8090"

help_info() {
  echo "submit tools"
  echo "usage:"
  echo "    [IMPORTANT] Before using this script, edit with it text editor, and put your app-key and secret-key in AK and SK"
  echo "    [IMPORTANT] Before submitting, you must commit your IMAGE to docker hub [https://hub.docker.com/], and make it public"
  echo "    To query result: ./submit.sh query"
  echo "    To submit image: ./submit.sh submit ImageName:ImageTag"
  echo ""
}

if [ "$1" == "submit" ]; then
  if [ -z "$2" ]; then
    echo "submit must have image url, using: ./submit.sh submit ImageName:ImageTag"
    exit 1
  fi
  # echo "./submit_client --app-key=$AK --secret-key=${SK} --server-host=${HOST1} --submit --docker-uri=$2"
  ./submit_client --app-key=$AK --secret-key=${SK} --server-host=${HOST1} --submit --docker-uri=$2
  if [ $? -ne 0 ]; then
    ./submit_client --app-key=$AK --secret-key=${SK} --server-host=${HOST2} --submit --docker-uri=$2
  fi
elif [ "$1" == "query" ]; then
  # echo "./submit_client --app-key=$AK --secret-key=${SK} --server-host=${HOST1}"
  ./submit_client --app-key=$AK --secret-key=${SK} --server-host=${HOST1}
  if [ $? -ne 0 ]; then
    ./submit_client --app-key=$AK --secret-key=${SK} --server-host=${HOST2}
  fi
else
  help_info
fi
