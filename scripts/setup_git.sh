#!/bin/sh

USERNAME=$1
EMAIL="$1@thi.de"

# Check length of provided username
[ ${#USERNAME} -ne 7 ] && echo "provided username does not have seven characters. exiting" && exit

echo ""
echo Setting email to $EMAIL
echo Setting username to $USERNAME

git config --global user.email $EMAIL
git config --global user.name $USERNAME

echo ""
echo New global git config:
git config --global --list
