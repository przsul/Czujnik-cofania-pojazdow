import sys, pyperclip

cmd = str(sys.argv[1])

result = "a0 "
checksum = 0

for c in cmd: 
	result += hex(ord(c))[2:] + " "
	checksum += ord(c)

result += "ff fe "
checksum += 0xFF + 0xFE
result += hex(checksum % 256)[2:]
result += " a0"

pyperclip.copy(result)

print(result.upper())
