MAIN_BRANCH="develop"
if [ -d ~/geppetto/ ]; then
    cd ~/geppetto/
    git switch $MAIN_BRANCH
    git pull
else
    cd ~
    git clone https://github.com/thomas-rou/Geppetto.git
    cd ~/geppetto/
    git switch $MAIN_BRANCH
    git pull
fi

./launch_local.sh