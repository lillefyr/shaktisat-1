cp ~/REPO/shaktiproject/software/shakti-sdk/software/examples/uart_applns/serialCommunication/* ~/REPO/shaktiproject/shaktisat-1/software/serialCommunication/
cd ~/REPO/shaktiproject/shaktisat-1/software/
git pull
git add *
if [ "$1" == "" ]
then
git commit -m "Copy of serialCommunication `date`"
else
git commit -m "$1"
fi
git push

cp ~/REPO/shaktiproject/software/shakti-sdk/software/examples/i2c_applns/amg88xx/* ~/REPO/shaktiproject/shaktisat-1/software/amg88xx
git pull
git add *
if [ "$1" == "" ]
then
git commit -m "Copy of amg88xx `date`"
else
git commit -m "$1"
fi
git push



echo "now go to macmini"
echo "cd /Users/asbjorn/arduino/REPO/sdey76/filetransfer/adventure/shaktisat1/software"

echo "./refreshAndPush.sh"
