#!/usr/bin/env python

def numberPointisValid(idNow, listID):
    temp = -1
    num = -1
    for i in range(len(listID)):
        if listID[i] != 0:
            temp = i
            if idNow == listID[i]:
                num = i    
        else:
            break
    
    # ham return tra ve | so luong id co nghia sau id can tim | vi tri cua id tim | so luong id co nghia |
    if temp == -1:
        return -1, 0, 0 # list rong
    elif num != -1:
        return temp - num, num, temp # so phan tu co nghia sau id follow
    else:
        return temp, num, temp # khong tim thay id trong list
    
def main():
    lisId = [2,3,4,0,0]
    print(numberPointisValid(5, lisId))

if __name__ == '__main__':
    main()
