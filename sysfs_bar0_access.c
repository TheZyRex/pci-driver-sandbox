#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <stdint.h>
#include <unistd.h>

// PCI BAR0 Offsets
#define ID_REGISTER        0x0		// Unique ID 
#define INV_REGISTER       0x4		// Invert Register -> value written to it, will be inverted
#define IRQ_REGISTER       0x8		// Interrupts
#define RANDVAL_REGISTER   0xc		// Random Value -> return rand value on any read


int main(int argc, char **argv)
{
  int fd, i;
  uint32_t config[5], offset;
  void *bar0;
  uint32_t *value;

  fd = open("/sys/bus/pci/devices/0000:00:02.0/resource0", O_RDWR | O_SYNC);
  if (fd < 0){
    perror("open");
    return EXIT_FAILURE;
  }

  bar0 = mmap(NULL, 64, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);

  close(fd);
  
  if (bar0 == MAP_FAILED) {
    perror("Memory mapping of BAR failed");
    return EXIT_FAILURE;
  }

  /* offset calculation */

  fd = open("/sys/bus/pci/devices/0000:00:02.0/config", O_RDONLY);
  if (fd < 0) {
    perror("open");
    return EXIT_FAILURE;
  }
  
  // read 20 bytes from the config file - at 0x14 we find our BAR0 address (see PCI Config Header)
  i = read(fd, config, 0x14); 

  if (i != 0x14) {
    perror("Error reading PCI config header");
    munmap(bar0, 64);
    return EXIT_FAILURE;
  }

  // offset calclation within a 4KB boundary - mmap usualy returns a 4kb alligned memory region
  offset = (config[4] & 0xfffffff0) % 4096;

  // adjust bar pointer
  bar0 = bar0 + offset;

  // access pci device
  value = (uint32_t*) (bar0 + RANDVAL_REGISTER);
  printf("pciecho-dev - random value: 0x%x\n", *value);

  value = (uint32_t*) (bar0 + INV_REGISTER);
  *value = 0x11223344;

  /* PCI device will invert the value and we will read the inverted value */
  printf("pciecho-dev - inverted value: 0x%x\n", *value);


  munmap(bar0, 64);

  return 0;
}