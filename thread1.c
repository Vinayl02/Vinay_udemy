#include<stdio.h>
#include<stdlib.h>

#include<pthread.h>
#include<unistd.h>  //for sleep() //used for Linux specific library


void *thread()
{
    printf("Message from Thread\n");
    //breakpoint(*thread);
    sleep(1);
}

void *test()
{
    printf("Message from Thread : Test function\n");
    sleep(2);
}

int main()
{
    pthread_t tid,t2;;
    if(pthread_create(&tid,NULL,&thread,NULL)!=0)
    {
        printf("failed to create thread\n");
        exit(1);
    }

    if(pthread_create(&t2,NULL,&test,NULL)!=0)
    {
        printf("failed to create thread\n");
        exit(1);
    }
    sleep(2);

    pthread_join(tid,NULL);
    pthread_join(t2,NULL);
    printf("Hello from Main program\n");
    return 0;
} 