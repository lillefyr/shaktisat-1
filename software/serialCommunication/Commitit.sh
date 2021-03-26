cp * ~/REPO/shaktiproject/shaktisat-1/software/serialCommunication/
cd ~/REPO/shaktiproject/shaktisat-1/software/serialCommunication/
git pull
git add *
git commit -m "Copy of serialCommunication `date`"
git push

echo "now go to macmini"
echo "cd /Users/asbjorn/arduino/REPO/sdey76/filetransfer/adventure/shaktisat1/software/serialCommunication"

echo "./refreshAndPush.sh"
