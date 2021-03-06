/* A simple server in the internet domain using TCP
   The port number is passed as an argument */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>

#define BUFFER_SIZE 204800

void error(const char *msg)
{
    perror(msg);
    exit(1);
}


void listn(int port) {
     int sockfd, newsockfd, portno;
     socklen_t clilen;
     char buffer[BUFFER_SIZE];
     struct sockaddr_in serv_addr, cli_addr;
     int n;
     sockfd = socket(AF_INET, SOCK_STREAM, 0);
     if (sockfd < 0) 
        error("ERROR opening socket");
     int yes = 1;
     if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int)) == -1 ) {
    	error("setsockopt");
	 }      
     bzero((char *) &serv_addr, sizeof(serv_addr));
     portno = port;
     serv_addr.sin_family = AF_INET;
     serv_addr.sin_addr.s_addr = INADDR_ANY;
     serv_addr.sin_port = htons(portno);
     
     
     
     if (bind(sockfd, (struct sockaddr *) &serv_addr,
              sizeof(serv_addr)) < 0) 
              error("ERROR on binding");
       
              
     listen(sockfd,5);  
     
     clilen = sizeof(cli_addr);
     newsockfd = accept(sockfd, 
                 (struct sockaddr *) &cli_addr, 
                 &clilen);
                 
             
     if (newsockfd < 0) 
          error("ERROR on accept");
          
     printf("Trying 2\n");
     while(1) {
    	 bzero(buffer,BUFFER_SIZE);
     	 n = read(newsockfd,buffer,BUFFER_SIZE-1); 
	   	 
    	 if (n < 0) 
	   		error("ERROR reading from socket");
	       	
   	 	printf("Message: \n%s\n",buffer);
   	 }
   	 
     
     shutdown(newsockfd,0);
     shutdown(sockfd,0);
     close(newsockfd);
     close(sockfd);
     printf("Closed sockets 2\n");   	 



}










int main(int argc, char *argv[])
{
     int sockfd, newsockfd, portno;
     socklen_t clilen;
     char buffer[BUFFER_SIZE];
     struct sockaddr_in serv_addr, cli_addr;
     int n;
     if (argc < 2) {
         fprintf(stderr,"ERROR, no port provided\n");
         exit(1);
     }
     sockfd = socket(AF_INET, SOCK_STREAM, 0);
     if (sockfd < 0) 
        error("ERROR opening socket");
     int yes = 1;
     if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int)) == -1 ) {
    	error("setsockopt");
	 }


        
        
     bzero((char *) &serv_addr, sizeof(serv_addr));
     portno = atoi(argv[1]);
     serv_addr.sin_family = AF_INET;
     serv_addr.sin_addr.s_addr = INADDR_ANY;
     serv_addr.sin_port = htons(portno);
     
     
     
     if (bind(sockfd, (struct sockaddr *) &serv_addr,
              sizeof(serv_addr)) < 0) 
              error("ERROR on binding");
     listen(sockfd,5);
     clilen = sizeof(cli_addr);
     newsockfd = accept(sockfd, 
                 (struct sockaddr *) &cli_addr, 
                 &clilen);
     if (newsockfd < 0) 
          error("ERROR on accept");
          
     int i=0;
     while(i < 10) {
	     bzero(buffer,BUFFER_SIZE);
	     n = read(newsockfd,buffer,BUFFER_SIZE-1); 
	   	 
	     if (n < 0) 
	    	error("ERROR reading from socket");
	    	
	    	
    	 printf("Message %d:\n%s\n",i,buffer);
    	 
    	 
    	 
    	 struct sockaddr_in sin;
		 socklen_t len = sizeof(sin);
    	 if (getsockname(sockfd, (struct sockaddr *)&sin, &len) == -1)
		    perror("getsockname");
    	 else
		    printf("port number %d\n", ntohs(sin.sin_port));

    	 
    	 char search[] = "http://192.168.0.100:";
    	 char* pos = strstr(buffer, search);
    	 
    	 char sub[6];
    	 memcpy(sub,pos + strlen(search) ,5);//(&buffer, pos, 5);
    	 sub[5] = '\0';
    	 int port = atoi(sub);


    	 
    	 
		 i++;
    }
     
     
    /*FILE *f = fopen("file.dat", "w");
	if (f == NULL)
	{
		printf("Error opening file!\n");
		exit(1);
	}     
    fwrite(buffer, 1, n, f);   
    fclose(f);*/
     
     
     shutdown(newsockfd,0);
     shutdown(sockfd,0);
     close(newsockfd);
     close(sockfd);
     printf("Closed sockets\n");
     return 0; 
}



