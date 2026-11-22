import sys, usb.core, struct
SET_CUR,GET_CUR,GET_MIN,GET_MAX,GET_DEF=0x01,0x81,0x82,0x83,0x87
def term_id(dev):
    # camera input terminal: VC interface extra bytes, descriptor subtype 0x02 (INPUT_TERMINAL), wTerminalType 0x0201
    for cfg in dev:
        for intf in cfg:
            if intf.bInterfaceClass==14 and intf.bInterfaceSubClass==1:
                e=bytes(intf.extra_descriptors); i=0
                while i<len(e):
                    l=e[i]
                    if e[i+1]==0x24 and e[i+2]==0x02 and e[i+4:i+6]==b'\x01\x02': return intf.bInterfaceNumber, e[i+3]
                    i+=l
def get(dev,ifn,tid,sel,n,req=GET_CUR): return bytes(dev.ctrl_transfer(0xA1,req,sel<<8,(tid<<8)|ifn,n))
def setc(dev,ifn,tid,sel,data): dev.ctrl_transfer(0x21,SET_CUR,sel<<8,(tid<<8)|ifn,data)
for dev in usb.core.find(find_all=True,idVendor=0x2c7f,idProduct=0x4a60):
    ifn,tid=term_id(dev)
    ae=get(dev,ifn,tid,0x02,1)[0]; ex=lambda r=GET_CUR: struct.unpack('<I',get(dev,ifn,tid,0x04,4,r))[0]
    print(f"bus {dev.bus} dev {dev.address}: ae_mode={ae} exposure cur={ex()} min={ex(GET_MIN)} max={ex(GET_MAX)} def={ex(GET_DEF)} (x100us)")
    if len(sys.argv)>1:
        setc(dev,ifn,tid,0x02,bytes([1])); setc(dev,ifn,tid,0x04,struct.pack('<I',int(sys.argv[1])))
        print("  -> ae_mode",get(dev,ifn,tid,0x02,1)[0],"exposure",ex())
