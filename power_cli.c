#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <signal.h>
#include <stdio.h>
#include <math.h>
#include <netinet/in.h> 
#include <sys/socket.h> 
#include <sys/wait.h> 
#include <fcntl.h> 
#include <getopt.h>
#include <poll.h>
#include <iio.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/ipc.h>
#include <sys/shm.h>


#pragma pack(1)
struct shm_seg
{
    int db;
    unsigned int counter;
    unsigned int freq;
    unsigned int gain;
};
#pragma pack(0)


#define SHM_SIZE sizeof(struct shm_seg)
char *shm_init(int key)
{
//    key_t key;
    int shmid;
    char *data;
    int mode;
    printf("* Creating SHM segment sith key %i and size %i bytes\n", key,  (int)SHM_SIZE);

    if ((shmid = shmget(key, SHM_SIZE, 0644|IPC_CREAT )) == -1) {
        perror("shmget");
        exit(1);
    }

    data = shmat(shmid, (void *)0, 0);
    if (data == (char *)(-1)) {
        perror("shmat");
        exit(1);
    }
    return data;
}


int main()
{
    int key = 100;
    struct shm_seg *seg = shm_init(key);
    double db_power =  (double)(seg->db); 

    // seg->freq = 1000; //  set freq to 1GHz
    // seg->gain = 10;   //  set hardware gain 10 dB
    db_power = db_power / 1000;
    printf("Power: %8.4f dB  Current Freq: %u MHz Counter: %u\n", db_power, seg->freq, seg->counter);
}