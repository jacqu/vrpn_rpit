/*
 * vrpn_rpit_server : server on target PC answering requests from
 *                    RPIt socket block. Reads Vicon VRPN stream. 
 * 
 * Compile with : make
 * 
 * JG, July 15 2019.
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <strings.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <netinet/tcp.h>
#include <netdb.h>
#include <signal.h>
#include <pthread.h>
#include <time.h>
#ifdef __MACH__
#include <mach/clock.h>
#include <mach/mach.h>
#endif
#include <vrpn_Connection.h>

// Defines for VRPN server
#define VRPN_SERVER_IP              "192.168.10.1"
#define VRPN_TARGET_NAME            "wiimote"
#define VRPN_TRACKER_TYPE           "vrpn_Tracker Pos_Quat"
#define VRPN_MSG_LEN                8
#define VRPN_TRANS_LEN              3
#define VRPN_ROT_LEN                4

// Check that these definitions are identical in client code
#define RPIT_SOCKET_CON_N           10        // Nb of double sent (control)
#define RPIT_SOCKET_MES_N           10        // Nb of double returned (measurement)
#define RPIT_SOCKET_PORT            "31415"   // Port of the server
#define RPIT_SOCKET_MES_PERIOD      2000      // Sampling period of the measurement (us)
#define RPIT_SOCKET_MAGIC           3141592   // Magic number
#define RPIT_SOCKET_WATCHDOG_TRIG   1000000   // Delay in us before watchdog is triggered

using namespace std;

// Struct definitions
struct RPIt_socket_mes_struct  {
  unsigned int        magic;                  // Magic number
  unsigned long long  timestamp;              // Absolute server time in ns 
  double              mes[RPIT_SOCKET_MES_N]; // Measurements
};

struct RPIt_socket_con_struct  {
  unsigned int        magic;                  // Magic number
  unsigned long long  timestamp;              // Absolute client time in ns
  double              con[RPIT_SOCKET_CON_N]; // Control signals
};

// Global variables
pthread_t                       mes_thread;
pthread_mutex_t                 mes_mutex;
struct RPIt_socket_mes_struct   mes;
struct RPIt_socket_con_struct   con;
unsigned char                   exit_req = 0;
vrpn_Connection                 *connection;

/*
 *  rpit_socket_get_time : get current absolute time in ns
 */
void rpit_socket_get_time( struct timespec *ts )  {

  if ( !ts )
    return;
  
  #ifdef __MACH__ // OS X does not have clock_gettime, use clock_get_time
  clock_serv_t cclock;
  mach_timespec_t mts;
  
  host_get_clock_service( mach_host_self(), CALENDAR_CLOCK, &cclock );
  clock_get_time( cclock, &mts );
  mach_port_deallocate( mach_task_self(), cclock );
  ts->tv_sec = mts.tv_sec;
  ts->tv_nsec = mts.tv_nsec;

  #else
  clock_gettime( CLOCK_MONOTONIC, ts );
  #endif
}

/*
 *  VRPN stream packet handler
 */
int rpit_vrpn_handler( void *userdata, vrpn_HANDLERPARAM p ) {
  int                       i;
  const char                *param = (p.buffer);
  static char               first_flag = 1;
  static unsigned long long first_time;
  unsigned long long        usec =  p.msg_time.tv_sec * 1e6 + 
                                    p.msg_time.tv_usec;

  if ( first_flag )  {  
         first_time = usec;
         first_flag = 0;
    }
  
  // Timestamp in us
  unsigned long long        timestamp = usec - first_time;

  // Check for errors
  if ( p.payload_len != ( VRPN_MSG_LEN * sizeof( vrpn_float64 ) ) ) {
    flockfile( stderr );
    fprintf(stderr,"vrpn_Tracker: change message payload error\n");
    fprintf(stderr,"             (got %d, expected %lud)\n",
      p.payload_len, 
      static_cast<unsigned long>( VRPN_MSG_LEN * sizeof(vrpn_float64) ) );
    funlockfile( stderr );
    return -1;
  }

  // Extract data
  
  // Enter critical section
  pthread_mutex_lock( &mes_mutex );
  
  // Define magic number
  mes.magic = RPIT_SOCKET_MAGIC;
  
  // Acquire server timestamp
  mes.timestamp = timestamp;
  
  // Acquire measurements
  for ( i = 0; i < VRPN_TRANS_LEN; i++ )
    mes.mes[i] = 
      ntohd( *( (vrpn_float64*)( param+((1+i)*sizeof(vrpn_float64)) ) ) );

  for ( i = 0; i < VRPN_ROT_LEN; i++ ) {
    mes.mes[i+VRPN_TRANS_LEN] = 
      ntohd( *( (vrpn_float64*)( param+((4+i)*sizeof(vrpn_float64)) ) ) );
  }
  
  // Exit critical section
  pthread_mutex_unlock( &mes_mutex );

  // Display pose
  flockfile( stderr );
  fprintf( stderr, 
    "Tracker : time us (%lli) pos (%lf,%lf,%lf) quat (%lf,%lf,%lf,%lf)\n",
    timestamp, 
    mes.mes[0], mes.mes[1], mes.mes[2],
    mes.mes[3], mes.mes[4], mes.mes[5], mes.mes[6] );
  funlockfile( stderr );
  
  return 0;
}

/*
 *  VRPN mainloop thread. 
 */
void *rpit_socket_server_update( void *ptr )  {
  
  while( 1 )  {
    
    // Check if exit is requested
    if ( exit_req )
      break;
      
    // Call to vrpn main loop
    connection->mainloop( );
      
  }
  
  return NULL;
}

/*
 *  SIGINT handler
 */
void rpit_socket_server_int_handler( int dummy )  {
  
  // Request termination of the thread
  exit_req = 1;
  
  // Wait for thread to terminate 
  pthread_join( mes_thread, NULL );
  
  // Cleanup
  flockfile( stderr );
  fprintf( stderr, "\nrpit_socket_server_int_handler: mainloop thread stopped. Cleaning up...\n" );
  funlockfile( stderr );
  
  // Exit
  exit( EXIT_SUCCESS );
}

/*
 *  Main.
 */
int main( void )  {
  
  struct addrinfo               hints;
  struct addrinfo               *result, *rp;
  int                           sfd, s, i;
  struct sockaddr_storage       peer_addr;
  socklen_t                     peer_addr_len;
  ssize_t                       nread;
  struct RPIt_socket_mes_struct  local_mes;
  struct RPIt_socket_con_struct  local_con;
  
  // VRPN initialization
  connection =  vrpn_get_connection_by_name( VRPN_SERVER_IP );

  long my_id = connection->register_sender( VRPN_TARGET_NAME );
  long my_type = connection->register_message_type( VRPN_TRACKER_TYPE );
  
  // Initialize mutex
  pthread_mutex_init( &mes_mutex, NULL );
  
  // Clear mes structure
  mes.timestamp = 0;
  for ( i = 0; i < RPIT_SOCKET_MES_N; i++ )
    mes.mes[i] = 0.0;
  
  // Clear con structure
  con.magic = 0;
  con.timestamp = 0;
  for ( i = 0; i < RPIT_SOCKET_CON_N; i++ )
    con.con[i] = 0.0;
  
  // Initialize SIGINT handler
  
  signal( SIGINT, rpit_socket_server_int_handler );
  
  memset( &hints, 0, sizeof( struct addrinfo ) );
  hints.ai_family = AF_UNSPEC;    /* Allow IPv4 or IPv6 */
  hints.ai_socktype = SOCK_DGRAM; /* Datagram socket */
  hints.ai_flags = AI_PASSIVE;    /* For wildcard IP address */
  hints.ai_protocol = 0;          /* Any protocol */
  hints.ai_canonname = NULL;
  hints.ai_addr = NULL;
  hints.ai_next = NULL;

  s = getaddrinfo( NULL, RPIT_SOCKET_PORT, &hints, &result );
  
  if ( s != 0 ) {
    flockfile( stderr );
    fprintf( stderr, "rpit_socket_server: function getaddrinfo returned: %s\n", gai_strerror( s ) );
    funlockfile( stderr );
    exit( EXIT_FAILURE );
   }
   
  /*   
    getaddrinfo() returns a list of address structures.
    Try each address until we successfully bind(2).
    If socket(2) (or bind(2)) fails, we (close the socket
    and) try the next address. 
  */

  for ( rp = result; rp != NULL; rp = rp->ai_next ) {
    sfd = socket( rp->ai_family, rp->ai_socktype, rp->ai_protocol );
    if ( sfd == -1 )
      continue;
    
    // Success
    if ( bind( sfd, rp->ai_addr, rp->ai_addrlen ) == 0 )
      break;

    close( sfd );
  }
  
  // No address succeeded
  if ( rp == NULL ) {
    flockfile( stderr );
    fprintf( stderr, "rpit_socket_server: could not bind. Aborting.\n" );
    funlockfile( stderr );
    exit( EXIT_FAILURE );
  }

  freeaddrinfo( result );
  
  // Start measurement thread and VRPN main loop
  connection->register_handler( my_type, rpit_vrpn_handler, (void*)"None", my_id );
  pthread_create( &mes_thread, NULL, rpit_socket_server_update, (void*) NULL );
  
  // Wait for control datagram and answer measurement to sender
  while ( 1 ) {
    
    // Read control signals from the socket
    peer_addr_len = sizeof( struct sockaddr_storage );
    nread = recvfrom(  sfd, (char*)&local_con, sizeof( struct RPIt_socket_con_struct ), 0,
                      (struct sockaddr *)&peer_addr, &peer_addr_len );
    
    // Memcopy is faster than socket read: avoid holding the mutex too long
    pthread_mutex_lock( &mes_mutex );
    
    memcpy( &con, &local_con, sizeof( struct RPIt_socket_con_struct ) );
    
    if ( nread == -1 )  {
      flockfile( stderr );
      fprintf( stderr, "rpit_socket_server: function recvfrom exited with error.\n" );
      funlockfile( stderr );
      
      // Clear control in case of error
      for ( i = 0; i < RPIT_SOCKET_CON_N; i++ )
        con.con[i] = 0.0;
    }
    
    if ( nread != sizeof( struct RPIt_socket_con_struct ) )  {
      flockfile( stderr );
      fprintf( stderr, "rpit_socket_server: function recvfrom did not receive the expected packet size.\n" );
      funlockfile( stderr );
      
      // Clear control in case of error
      for ( i = 0; i < RPIT_SOCKET_CON_N; i++ )
        con.con[i] = 0.0;
    }
                    
    if ( con.magic != RPIT_SOCKET_MAGIC )  {
      flockfile( stderr );
      fprintf( stderr, "rpit_socket_server: magic number problem. Expected %d but received %d.\n", RPIT_SOCKET_MAGIC, con.magic );
      funlockfile( stderr );
      
      // Clear control in case of error
      for ( i = 0; i < RPIT_SOCKET_CON_N; i++ )
        con.con[i] = 0.0;
    }
    
    pthread_mutex_unlock( &mes_mutex );
    
    // Critical section : copy of the measurements to a local variable
    pthread_mutex_lock( &mes_mutex );
    memcpy( &local_mes, &mes, sizeof( struct RPIt_socket_mes_struct ) );
    pthread_mutex_unlock( &mes_mutex );  
    
    // Send measurements to the socket
    if ( sendto(  sfd, (char*)&local_mes, sizeof( struct RPIt_socket_mes_struct ), 0,
                  (struct sockaddr *)&peer_addr,
                  peer_addr_len) != sizeof( struct RPIt_socket_mes_struct ) )  {
      flockfile( stderr );
      fprintf( stderr, "rpit_socket_server: error sending measurements.\n" );
      funlockfile( stderr );
    }    
  }
    
  exit( EXIT_SUCCESS );
}
