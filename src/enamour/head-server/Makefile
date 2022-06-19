.PHONY = all clean

CC := g++

CFLAGS := -g -Wall

LDFLAGS := -pthread -lpigpio

SRCS := ./src/head-server.cpp

BINS := ./bin/head-server

all: head-server

head-server:
	${CC} ${CFLAGS} -o ${BINS} ${SRCS} ${LDFLAGS}

clean:
	rm ${BINS}

install: head-server
ifneq ($(shell id -u), 0)
	@echo "You must be root to perform this action."
else
	install -d /opt/head-server
	install -m 644 bin/head-server /opt/head-server/
	chmod +x /opt/head-server/head-server
	install -m 777 head-server.service /etc/systemd/system/
endif
	systemctl daemon-reload
	systemctl stop head-server.service
	systemctl enable head-server.service
	systemctl start head-server.service