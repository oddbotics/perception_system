CC = g++
LINK = g++
INSTALL = install
CFLAGS = -Wall `pkg-config opencv --cflags` -I /usr/include/boost-1_46 -I.
LFLAGS = -Wall `pkg-config opencv --libs` -L /usr/lib  -lboost_system -lboost_filesystem -lopencv_features2d -lopencv_nonfree -lopencv_ocl -lopencv_highgui -lopencv_core -lpthread
#OBJS = car.o cartracker.o

all: bottracker


bot_tracker.o: bot_tracker.cpp bot.h
	$(CC) $(CFLAGS) -c $^ 

bot.o: bot.h


bottracker: bot_tracker.o bot.o
	$(LINK) $^ -o $@  $(LFLAGS)

#test.o: test.cpp car.h
#	$(CC) $(CFLAGS) -c $^ 

#test: test.o car.o
#	$(LINK) $^ -o $@  $(LFLAGS)


clean:
	rm -f  *.o *.gch bottracker*

install:
	cp bottracker /usr/bin/

