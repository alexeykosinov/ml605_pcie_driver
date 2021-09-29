#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <malloc.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdint.h>

#define DEVICE "/dev/ml605_pcie-0"
#define MEM_BUFFER_SIZE 16
int fd = 0;


int write_device() {
    ssize_t ret;
    char *data = (char *)malloc(1024 * sizeof(char));
    printf("Enter the data to write into device\n");
    scanf(" %[^\n]", data);
    printf("Total length is %d B\n", strlen(data));
    ret = write(fd, data, strlen(data));

    if (ret == -1) printf("writting failed\n");

    free(data);
    return 0;
}

int read_device() {
    int read_length = 0;
    ssize_t ret;
    char *data = (char *)malloc(1024 * sizeof(char));
    printf("enter the length of the buffer to read\n");
    scanf("%d", &read_length);
    printf("the read length selected is %d\n", read_length);
    memset(data, fd, sizeof(data));
    data[0] = '\0';
    ret = read(0, data, read_length);
    printf("DEVICE_READ : %s\n", data);

    if(ret == -1) printf("reading failed\n");

    free(data);
    return 0;
}

int main() {
    int i = 0;
    int value = 0;
    uint32_t *shm = NULL;

    if (access(DEVICE, F_OK) == -1) {
        printf("module %s not loaded\n", DEVICE);
        return -1;
    }

    while(1) {
        printf("\tplease enter 1 to write\n");
        printf("\tplease enter 2 to read\n");
        printf("\tplease enter 3 mmap perform\n");

        scanf("%d", &value);

        switch(value) {
            case 1:
                fd = open(DEVICE, O_RDWR);
                write_device();
                close(fd);
                break;
            case 2:
                fd = open(DEVICE, O_RDWR);
                read_device();
                close(fd);
                break;
            case 3:
                fd = open(DEVICE, O_RDWR);

                shm = mmap(NULL, MEM_BUFFER_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);
                if (MAP_FAILED == shm) {
                    printf("mmap: %s\n", strerror(errno));
                    return -1;
                }

                for (i = 0; i < MEM_BUFFER_SIZE; i++){
                    printf("DATA = 0x%08X\n", shm[i]);
                }

                // strcpy(shm, "User write to share memory!");
                // printf("After write, shm = %s\n", shm);
                munmap(shm, MEM_BUFFER_SIZE);
                close(fd);  
                break;
            default: 
                printf("Unknown option selected, please enter right one\n");
                break;
        }
    }
    return 0;
}