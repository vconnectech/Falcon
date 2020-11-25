#include <stdio.h> 
#include <netdb.h> 
#include <netinet/in.h> 
#include <stdlib.h> 
#include <string.h> 
#include <sys/socket.h> 
#include <sys/types.h> 
#include <linux/can.h>
#include <sys/ioctl.h>
#include <net/if.h>

int main(int argc, char **argv){

    struct ifreq ifr;	
    struct sockaddr_can addr;
    struct can_frame frame;
    int s;
    
    memset(&ifr, 0x0, sizeof(ifr));
    memset(&addr, 0x0, sizeof(addr));
    memset(&frame, 0x0, sizeof(frame));
    
    /* open CAN_RAW socket */
    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    
    /* convert interface sting "can0" into interface index */
    strcpy(ifr.ifr_name, "can0");
    
    ioctl(s, SIOCGIFINDEX, &ifr);
    
    /* setup address for bind */
    addr.can_ifindex = ifr.ifr_ifindex;
    addr.can_family= PF_CAN;
    
    /* bind socket to the can0 interface */
    bind(s, (struct sockaddr *)&addr, sizeof(addr));
    
    /* first fill, then send the CAN frame */
    frame.can_id = 0x23;
    strcpy((char *)frame.data, "hello");
    
    frame.can_dlc = 5;
    write(s, &frame, sizeof(frame));
    
    close(s);
    return 0;
}	
