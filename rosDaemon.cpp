/* 
 * tcpserver.c - A simple TCP echo server 
 * usage: tcpserver <port>
 */

#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <netdb.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <boost/process.hpp>
#define BUFSIZE 1024

using namespace std;
#if 0
/* 
 * Structs exported from in.h
 */

/* Internet address */
struct in_addr {
  unsigned int s_addr; 
};

/* Internet style socket address */
struct sockaddr_in  {
  unsigned short int sin_family; /* Address family */
  unsigned short int sin_port;   /* Port number */
  struct in_addr sin_addr;	 /* IP address */
  unsigned char sin_zero[...];   /* Pad to size of 'struct sockaddr' */
};

/*
 * Struct exported from netdb.h
 */

/* Domain name service (DNS) host entry */
struct hostent {
  char    *h_name;        /* official name of host */
  char    **h_aliases;    /* alias list */
  int     h_addrtype;     /* host address type */
  int     h_length;       /* length of address */
  char    **h_addr_list;  /* list of addresses */
}
#endif

/*
 * error - wrapper for perror
 */
void error(char *msg) {
  perror(msg);
  exit(1);
}

int main(int argc, char **argv) {
  int parentfd; /* parent socket */
  int childfd; /* child socket */
  int portno; /* port to listen on */
  socklen_t clientlen; /* byte size of client's address */
  struct sockaddr_in serveraddr; /* server's addr */
  struct sockaddr_in clientaddr; /* client addr */
  struct hostent *hostp; /* client host info */
  char buf[BUFSIZE]; /* message buffer */
  char *hostaddrp; /* dotted decimal host addr string */
  int optval; /* flag value for setsockopt */
  int n; /* message byte size */
  system("echo \"\" > sys.log");


  portno = 7777;
  parentfd = socket(AF_INET, SOCK_STREAM, 0);
  if (parentfd < 0) 
    system("echo \"Error opening socket\" >> sys.log");

  /* setsockopt: Handy debugging trick that lets 
   * us rerun the server immediately after we kill it; 
   * otherwise we have to wait about 20 secs. 
   * Eliminates "ERROR on binding: Address already in use" error. 
   */
  optval = 1;
  setsockopt(parentfd, SOL_SOCKET, SO_REUSEADDR, 
	     (const void *)&optval , sizeof(int));

  /*
   * build the server's Internet address
   */
  bzero((char *) &serveraddr, sizeof(serveraddr));

  /* this is an Internet address */
  serveraddr.sin_family = AF_INET;

  /* let the system figure out our IP address */
  serveraddr.sin_addr.s_addr = htonl(INADDR_ANY);

  /* this is the port we will listen on */
  serveraddr.sin_port = htons((unsigned short)portno);

  /* 
   * bind: associate the parent socket with a port 
   */
  if (bind(parentfd, (struct sockaddr *) &serveraddr, 
	   sizeof(serveraddr)) < 0) 
    system("echo \"Error binding socket\" >> sys.log");

  /* 
   * listen: make this socket ready to accept connection requests 
   */
  if (listen(parentfd, 5) < 0) /* allow 5 requests to queue up */ 
    system("echo \"Error listening to socket\" >> sys.log");

  /* 
   * main loop: wait for a connection request, echo input line, 
   * then close connection.
   */
  clientlen = sizeof(clientaddr);
  while (1) {

    /* 
     * accept: wait for a connection request 
     */
    childfd = accept(parentfd, (struct sockaddr *) &clientaddr, &clientlen);
    if (childfd < 0) 
      system("echo \"Error on accept\" >> sys.log");
    
    /* 
     * gethostbyaddr: determine who sent the message 
     */
    hostp = gethostbyaddr((const char *)&clientaddr.sin_addr.s_addr, 
			  sizeof(clientaddr.sin_addr.s_addr), AF_INET);
    if (hostp == NULL)
      system("echo \"Error on gethostbyaddr\" >> sys.log");
    hostaddrp = inet_ntoa(clientaddr.sin_addr);
    if (hostaddrp == NULL)
      system("echo \"Error on inet_ntoa\" >> sys.log");
    printf("server established connection with %s (%s)\n", 
	   hostp->h_name, hostaddrp);
	   system("echo \"PICMD: Client connected.\" >> sys.log");
    
    /* 
     * read: read input string from the client
     */
    bzero(buf, BUFSIZE);
    n = read(childfd, buf, BUFSIZE);
    if (n < 0) 
      printf("ERROR reading from socket");
    
    if(!strcmp(buf,"PIHW_ON\n")){
        printf("PICMD: Booting pihw node.\n");
        
        std::string executable_name = "rosrun";
        std::vector<std::string> args;
        args.push_back("minho_team_pihw"); args.push_back("pihw");
        std::string exe = boost::process::find_executable_in_path(executable_name); 
        boost::process::create_child(exe,args);
        
        system("echo \"PICMD: Booted pihw node.\" >> sys.log");
    } else if(!strcmp(buf,"PIHW_OFF\n")){
        printf("PICMD: Killing pihw node.\n");
          
        std::string executable_name = "pkill";
        std::vector<std::string> args;
        args.push_back("-f"); args.push_back("pihw");
        std::string exe = boost::process::find_executable_in_path(executable_name); 
        boost::process::create_child(exe,args);
        
        system("echo \"PICMD: Killed pihw node.\" >> sys.log"); 
    } else { printf("PICMD: Unknown command.\n"); system("echo \"PICMD: Unknown command.\" >> sys.log"); }

    close(childfd);
  }
}
