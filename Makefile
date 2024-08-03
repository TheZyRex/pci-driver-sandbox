obj-m += echodev-drv.o

all:
	make -C ../linux-6.6.43 M=${PWD} modules

clean:
	make -C ../linux-6.6.43 M=${PWD} clean