#include <iostream>;
using namespace std;

int main()
{
    int n; // so nguyen n nhap tu ban phim
    int giaithua = 1;
    cout << "nhap vao so nguyen n de tinh giai thua: ";
    cin >> n;
    if (n == 0) 
    {
        giaithua = 1;
        cout << giaithua << endl;
    }
   else
   {
   for ( int i = 1 ; i <=n; i++) 
   {
   giaithua = giaithua*i; 
   }
    cout << "giai thua so" << n << "la" << giaithua << endl;
   }    
return 0;
}