all: clean Debug Release

Debug: Debug_Sample_01 Debug_Sample_02 Debug_Sample_03

Debug_Sample_01:
	g++ -fpermissive -o ./x64/Debug/Sample_01 ./Sample_01/Main_01.cpp ./src/GJK_EPA.cpp
Debug_Sample_02:
	g++ -fpermissive -o ./x64/Debug/Sample_02 ./Sample_02/Main_02.cpp ./src/GJK_EPA.cpp
Debug_Sample_03:
	g++ -fpermissive -o ./x64/Debug/Sample_03 ./Sample_03/Main_03.cpp ./src/GJK_EPA.cpp 
	
Release: Release_Sample_01 Release_Sample_02 Release_Sample_03

Release_Sample_01:
	g++ -fpermissive -o ./x64/Release/Sample_01 ./Sample_01/Main_01.cpp ./src/GJK_EPA.cpp -Ofast
Release_Sample_02:
	g++ -fpermissive -o ./x64/Release/Sample_02 ./Sample_02/Main_02.cpp ./src/GJK_EPA.cpp -Ofast
Release_Sample_03:
	g++ -fpermissive -o ./x64/Release/Sample_03 ./Sample_03/Main_03.cpp ./src/GJK_EPA.cpp -Ofast
	
clean: clean_Debug clean_Release

clean_Debug:
	rm x64/Debug -r
	mkdir x64/Debug
	
clean_Release:
	rm x64/Release -r
	mkdir x64/Release