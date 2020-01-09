all: clean create_bin
	g++ -fpermissive -o ./bin/Sample_01 ./Sample_01/Main_01.cpp ./src/GJK_EPA.cpp -O3
	g++ -fpermissive -o ./bin/Sample_02 ./Sample_02/Main_02.cpp ./src/GJK_EPA.cpp -O3
	g++ -fpermissive -o ./bin/Sample_03 ./Sample_03/Main_03.cpp ./src/GJK_EPA.cpp -O3
	
clean: 
	rm bin -r -rf

create_bin:	
	mkdir bin -p