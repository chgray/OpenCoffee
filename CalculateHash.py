import sys
import hashlib
import binascii

class ChecksumContent(object):
    
    def __init__(self, content):
        print("ChecksumContent : %d" % len(content))
         
        self.shaHash = hashlib.sha256(content)
        self.hexHash = binascii.hexlify(self.shaHash.digest())
        self.hexHash = self.hexHash.decode('utf-8')
        self.content = content  
        
        print(self.Hash())  
        
    def Content(self):
        return self.content
        
    def Hash(self):
        return self.hexHash
    
    
# with open("file.txt", 'rb') as f:
#     ret = ChecksumContent(f.read())
#     print (ret.Hash())
 
n = len(sys.argv)
for i in range(1, n):
    print(sys.argv[i], end = " ")
    
    with open(sys.argv[i], 'rb') as f:
        ret = ChecksumContent(f.read())
        print (ret.Hash())
