# Définition des raccourcis :

all: pwm_rt

.PHONY: tout


# Crétation des exécutables :

pwm_rt: pwm_rt.o
	gcc $^ -o $@ $(shell /usr/xenomai/bin/xeno-config --skin native --ldflags)


# Création des objets :

%.o: %.c
	gcc -c $^ -o $@ $(shell /usr/xenomai/bin/xeno-config --skin native  --cflags)


# Suppression des objets :

clean:
	rm *.o

.PHONY: clean
