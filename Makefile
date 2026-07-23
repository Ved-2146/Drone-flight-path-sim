CC = gcc
CFLAGS = -Wall
TARGET = drone_sim.exe
OBJS = main.o grid.o drone.o pathfinding.o

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $(TARGET) $(OBJS)

main.o: main.c grid.h drone.h pathfinding.h
	$(CC) $(CFLAGS) -c main.c

grid.o: grid.c grid.h
	$(CC) $(CFLAGS) -c grid.c

drone.o: drone.c drone.h grid.h
	$(CC) $(CFLAGS) -c drone.c

pathfinding.o: pathfinding.c pathfinding.h grid.h drone.h
	$(CC) $(CFLAGS) -c pathfinding.c

run: all
	$(TARGET)

clean:
	-del /Q *.o $(TARGET)
