export EE4308WS=`echo "$( cd "$( dirname "$0" )" && pwd )"`
cd `echo $EE4308WS`

rm -rf devel build src/CMakeLists.txt

source make.sh
