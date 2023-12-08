#include <ros/ros.h>
#include <std_msgs/Char.h>
#include <std_msgs/Int16.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <map>

    char key(' ');

    // For non-blocking keyboard inputs
    int getch(void)
    {
        int ch;
        struct termios oldt;
        struct termios newt;

        // Store old settings, and copy to new settings
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;

        // Make required changes and apply the settings
        newt.c_lflag &= ~(ICANON | ECHO);
        newt.c_iflag |= IGNBRK;
        newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
        newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
        newt.c_cc[VMIN] = 1;
        newt.c_cc[VTIME] = 0;
        tcsetattr(fileno(stdin), TCSANOW, &newt);

        // Get the current character
        ch = getchar();

        // Reapply old settings
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

        return ch;
    }

int main(int argc, char** argv){
    // init
        ros::init(argc, argv, "key_pub");
        ros::NodeHandle n;
        ros::Publisher pub1 = n.advertise<std_msgs::Char>("keypub_char",10);
        ros::Publisher pub2 = n.advertise<std_msgs::Int16>("keypub_int",10);
        ros::Rate r(100.0);
        std_msgs::Char a ;
        std_msgs::Int16 b ;

    while(n.ok()){
        ros::spinOnce();

        key = getch();  
        a.data = key ;
        b.data = key;

        // printf("key_char =%c \n", a.data);
        // printf("key_int  =%d \n", b.data);

        pub1.publish(a);
        pub2.publish(b);

        if (key == '\x03')
        {        
            printf("\n\n                 Da luu data in file : data_tag.json              \n\n");
            break;
        }
        
        r.sleep();
    }
    return 0;
}