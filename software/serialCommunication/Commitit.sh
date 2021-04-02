cp * ~/REPO/shaktiproject/shaktisat-1/software/serialCommunication/
cd ~/REPO/shaktiproject/shaktisat-1/software/serialCommunication/
git pull
git add *
if [ "$1" == "" ]
then
git commit -m "Copy of serialCommunication `date`"
else
git commit -m "$1"
fi
git push

echo "now go to macmini"
echo "cd /Users/asbjorn/arduino/REPO/sdey76/filetransfer/adventure/shaktisat1/software/serialCommunication"

echo "./refreshAndPush.sh"
