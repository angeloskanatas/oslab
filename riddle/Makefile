.PHONY: solutions reset tier2

kill: kill.c
	gcc -o kill kill.c

esp: esp.c
	gcc -o esp esp.c

change: change.c
	gcc -o change change.c

the_One: the_One.c
	gcc -o the_One the_One.c

solutions: kill esp change the_One
	mv ./solutions/challenge* ./

reset:
	rm -f kill .hello_there .hey_there pipe* magic_mirror riddle.savegame bf0* found_secret_number esp change the_One tier2.so library.o
	mv challenge* ./solutions
