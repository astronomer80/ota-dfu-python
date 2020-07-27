#!/usr/bin/env python
"""
------------------------------------------------------------------------------
 DFU Server for Nordic nRF52 based systems.
 Conforms to nRF52_SDK 11.0 BLE_DFU requirements.
------------------------------------------------------------------------------
"""
import os, re
import sys
import pexpect
import optparse
import time

from intelhex import IntelHex
from array    import array
from unpacker import Unpacker

VERBOSE=1

# DFU Opcodes
class Commands:
    START_DFU                    = 1
    INITIALIZE_DFU               = 2
    RECEIVE_FIRMWARE_IMAGE       = 3
    VALIDATE_FIRMWARE_IMAGE      = 4
    ACTIVATE_FIRMWARE_AND_RESET  = 5
    SYSTEM_RESET                 = 6
    PKT_RCPT_NOTIF_REQ           = 8

# DFU Procedures values
DFU_proc_to_str = {
    "01" : "START",
    "02" : "INIT",
    "03" : "RECEIVE_APP",
    "04" : "VALIDATE",
    "08" : "PKT_RCPT_REQ",
}

# DFU Operations values
DFU_oper_to_str = {
    "01" : "START_DFU",
    "02" : "RECEIVE_INIT",
    "03" : "RECEIVE_FW",
    "04" : "VALIDATE",
    "05" : "ACTIVATE_N_RESET",
    "06" : "SYS_RESET",
    "07" : "IMAGE_SIZE_REQ",
    "08" : "PKT_RCPT_REQ",
    "10" : "RESPONSE",
    "11" : "PKT_RCPT_NOTIF",
}

# DFU Status values
DFU_status_to_str = {
    "01" : "SUCCESS",
    "02" : "INVALID_STATE",
    "03" : "NOT_SUPPORTED",
    "04" : "DATA_SIZE",
    "05" : "CRC_ERROR",
    "06" : "OPER_FAILED",
}

class UUID:
    CCCD 				= "00002902-0000-1000-8000-00805f9b34fb"
    DFU_Control_Point 	= "00001531-1212-efde-1523-785feabcd123"
    DFU_Packet			= "00001532-1212-efde-1523-785feabcd123"
    DFU_Version			= "00001534-1212-efde-1523-785feabcd123"

"""
------------------------------------------------------------------------------
 Convert a number into an array of 4 bytes (LSB).
 This has been modified to prepend 8 zero bytes per the new DFU spec.
------------------------------------------------------------------------------
"""
def convert_uint32_to_array(value):
    return [0,0,0,0,0,0,0,0,
           (value >> 0  & 0xFF),
           (value >> 8  & 0xFF),
           (value >> 16 & 0xFF),
           (value >> 24 & 0xFF)
    ]

"""
------------------------------------------------------------------------------
 Convert a number into an array of 2 bytes (LSB).
------------------------------------------------------------------------------
"""
def convert_uint16_to_array(value):
    return [
        (value >> 0 & 0xFF),
        (value >> 8 & 0xFF)
    ]

"""
------------------------------------------------------------------------------

------------------------------------------------------------------------------
"""
def convert_array_to_hex_string(arr):
    hex_str = ""
    for val in arr:
        if val > 255:
            raise Exception("Value is greater than it is possible to represent with one byte")
        hex_str += "%02x" % val

    return hex_str

def debug_msg(message):
	"""
	Print messages in verbose mode
	"""
	if VERBOSE:
		print "[DEBUG]" + message


	
"""
------------------------------------------------------------------------------
 Define the BleDfuServer class
------------------------------------------------------------------------------
"""
class BleDfuServer(object):
    """
    #--------------------------------------------------------------------------
    # Adjust these handle values to your peripheral device requirements.
    #--------------------------------------------------------------------------
    """
    ctrlpt_handle      = 0x10
    ctrlpt_cccd_handle = 0x11
    data_handle        = 0x0e
    reset_handle      = 0x13
    ctrlpt_cccd_handle_buttonless  = 0x14

    #TODO Check this parameter to speed up the rprocedure
    pkt_receipt_interval = 10 #DEFAULT=10
    pkt_payload_size     = 20 #DEFAULT=20

    value_written_success_msg='Characteristic value was written successfully'
    value_written_success_msg_alt='.* Characteristic value was written successfully'

    """
    --------------------------------------------------------------------------
    
    --------------------------------------------------------------------------
    """
    def __init__(self, target_mac, hexfile_path, datfile_path):
		debug_msg("Init")
		self.target_mac = target_mac

		self.hexfile_path = hexfile_path
		self.datfile_path = datfile_path

		self.bluez_version=pexpect.run ('bluetoothd -v').rstrip()
		debug_msg("BlueZ version: " + self.bluez_version)

		#Send gatttoll command
		self.ble_conn = pexpect.spawn("gatttool -b '%s' -t random --interactive" % target_mac)

		# remove next line comment for pexpect detail tracing.
		#TODO reuse this
		#self.ble_conn.logfile = sys.stdout

    
    def ble_msg_verbose(self):
    	"""
		Print messages if VERBOSE mode is enabled
    	"""
        if VERBOSE:    
            msg_ret = self.ble_conn.before
            if msg_ret!="":
                print "[DEBUG]" + msg_ret


    """
    --------------------------------------------------------------------------
     Connect to peripheral device.
    --------------------------------------------------------------------------
    """
    def scan_and_connect(self):
		debug_msg("scan_and_connect")

		debug_msg("connect")
		self.ble_conn.sendline('connect')

		if(float(self.bluez_version)>=5.4):
			debug_msg("BlueZ 5.4")
			try:
				
				#For BlueZ >5.4
				self.ble_conn.expect('\[CON\].*>', timeout=10)                
			except pexpect.TIMEOUT, e:
				try:
					self.ble_conn.expect('\[LE\]>', timeout=10)
				except pexpect.TIMEOUT, e:
					print "[scan_and_connect] Error: Connection timeout 2. \nMake sure that bluez version 5.40 and later is installed"
				self.ble_msg_verbose()
				return False
		else:
			debug_msg("BlueZ 5.3")
			#For BlueZ >5.3
			try:
				res = self.ble_conn.expect('Connection successful', timeout=10)
			except pexpect.TIMEOUT, e:
				print "[scan_and_connect] Error: Connection timeout 3"
				return False

                ctrlpt_cccd_handle_buttonless, reset_handle = getDFUControlPointHandle(self.ble_conn, UUID.CCCD)
                self.ctrlpt_cccd_handle_buttonless = int(ctrlpt_cccd_handle_buttonless, 16)
                self.reset_handle = int(reset_handle, 16)
     			
		debug_msg("END scan_and_connect")
		return True        
    """
    --------------------------------------------------------------------------
     Wait for notification to arrive.
     Example format: "Notification handle = 0x0019 value: 10 01 01"
    --------------------------------------------------------------------------
    """
    def _dfu_wait_for_notify(self):

        while True:
            #print "dfu_wait_for_notify"

            if not self.ble_conn.isalive():
                print "connection not alive"
                return None

            try:
                index = self.ble_conn.expect('Notification handle = .*? \r\n', timeout=30)

            except pexpect.TIMEOUT:
                #
                # The gatttool does not report link-lost directly.
                # The only way found to detect it is monitoring the prompt '[CON]'
                # and if it goes to '[   ]' this indicates the connection has
                # been broken.
                # In order to get a updated prompt string, issue an empty
                # sendline('').  If it contains the '[   ]' string, then
                # raise an exception. Otherwise, if not a link-lost condition,
                # continue to wait.
                #
                self.ble_conn.sendline('')
                string = self.ble_conn.before
                if '[   ]' in string:
                    print 'Connection lost! '
                    raise Exception('Connection Lost')
                return None
			
			#Print messages if VERBOSE mode is enabled
            self.ble_msg_verbose()
	    
            if index == 0:
                after = self.ble_conn.after
		debug_msg("After:" + str(after))
                hxstr = after.split()[3:]
                handle = long(float.fromhex(hxstr[0]))
		debug_msg("HAndle:" + str(handle))
                return hxstr[2:]

            else:
                print "unexpeced index: {0}".format(index)
                return None

    """
    --------------------------------------------------------------------------
     Parse notification status results
    --------------------------------------------------------------------------
    """
    def _dfu_parse_notify(self, notify):

        if len(notify) < 3:
            print "notify data length error"
            return None

        dfu_oper = notify[0]
        oper_str = DFU_oper_to_str[dfu_oper]

        debug_msg("_dfu_parse_notify:" + str(notify) + " dfu_oper:" + str(dfu_oper))

        if oper_str == "RESPONSE":
            dfu_process = notify[1]
            dfu_status  = notify[2]

            process_str = DFU_proc_to_str[dfu_process]
            status_str  = DFU_status_to_str[dfu_status]

            debug_msg(str("oper: {0}, proc: {1}, status: {2}".format(oper_str, process_str, status_str)))

            if oper_str == "RESPONSE" and status_str == "SUCCESS":
                return "OK"
            else:
                print "ERROR: [_dfu_parse_notify]"
                sys.exit(1)
                return "FAIL"


        if oper_str == "PKT_RCPT_NOTIF":

            byte1 = int(notify[4], 16)
            byte2 = int(notify[3], 16)
            byte3 = int(notify[2], 16)
            byte4 = int(notify[1], 16)

            receipt = 0
            receipt = receipt + (byte1 << 24)
            receipt = receipt + (byte2 << 16)
            receipt = receipt + (byte3 << 8)
            receipt = receipt + (byte4 << 0)

            print "PKT_RCPT: {0:8}".format(receipt) + " of " + str(self.hex_size)

            return "OK"


    """
    --------------------------------------------------------------------------
     Send two bytes: command + option
    --------------------------------------------------------------------------
    """
    def _dfu_state_set(self, opcode):
		debug_msg(str('char-write-req 0x%04x %04x' % (self.ctrlpt_handle, opcode)))
		self.ble_conn.sendline('char-write-req 0x%04x %04x' % (self.ctrlpt_handle, opcode))

		# Verify that command was successfully written
		try:
			res = self.ble_conn.expect(self.value_written_success_msg, timeout=10)
		except pexpect.TIMEOUT, e:
			print "ERROR: _dfu_state_set State timeout"

		self.ble_msg_verbose()

    #--------------------------------------------------------------------------
    # Send one byte: command
    #--------------------------------------------------------------------------
    def _dfu_state_set_byte(self, opcode):
        self.ble_conn.sendline('char-write-req 0x%04x %02x' % (self.ctrlpt_handle, opcode))

        # Verify that command was successfully written
        try:
            res = self.ble_conn.expect(self.value_written_success_msg, timeout=10)
        except pexpect.TIMEOUT, e:
            print "ERROR: State timeout"
            sys.exit(1)

        #Print messages if VERBOSE mode is enabled
		#ble_msg_verbose()

    #--------------------------------------------------------------------------
    # Send 3 bytes: PKT_RCPT_NOTIF_REQ with interval of 10 (0x0a)
    #--------------------------------------------------------------------------
    def _dfu_pkt_rcpt_notif_req(self):

        opcode = 0x080000
        opcode = opcode + (self.pkt_receipt_interval << 8)

        self.ble_conn.sendline('char-write-req 0x%04x %06x' % (self.ctrlpt_handle, opcode))

        # Verify that command was successfully written
        try:
            res = self.ble_conn.expect(self.value_written_success_msg, timeout=10)
        except pexpect.TIMEOUT, e:
            print "Send PKT_RCPT_NOTIF_REQ timeout"

        #Print messages if VERBOSE mode is enabled
		#ble_msg_verbose()

    #--------------------------------------------------------------------------
    # Send an array of bytes: request mode
    #--------------------------------------------------------------------------
    def _dfu_data_send_req(self, data_arr):
        hex_str = convert_array_to_hex_string(data_arr)
        #print hex_str
        self.ble_conn.sendline('char-write-req 0x%04x %s' % (self.data_handle, hex_str))

        # Verify that data was successfully written
        try:
            res = self.ble_conn.expect(self.value_written_success_msg, timeout=10)
        except pexpect.TIMEOUT, e:
            print "Data timeout"

	
	#Print messages if VERBOSE mode is enabled
	#ble_msg_verbose()
	

    #--------------------------------------------------------------------------
    # Send an array of bytes: command mode
    #--------------------------------------------------------------------------
    def _dfu_data_send_cmd(self, data_arr):
        hex_str = convert_array_to_hex_string(data_arr)
        #print hex_str
        self.ble_conn.sendline('char-write-cmd 0x%04x %s' % (self.data_handle, hex_str))

    #--------------------------------------------------------------------------
    # Enable DFU Control Point CCCD (Notifications)
    #--------------------------------------------------------------------------
    def _dfu_enable_cccd(self, alreadyDfuMode):
		handle=self.ctrlpt_cccd_handle
		if(alreadyDfuMode==False):
		   handle=self.ctrlpt_cccd_handle_buttonless
		
		debug_msg("_dfu_enable_cccd")
		cccd_enable_value_array_lsb = convert_uint16_to_array(0x0001)
		cccd_enable_value_hex_string = convert_array_to_hex_string(cccd_enable_value_array_lsb)
		command=str('char-write-req 0x%04x %s' % (handle, cccd_enable_value_hex_string))
		debug_msg(command)
		self.ble_conn.sendline(command)

		if(alreadyDfuMode==False):
			# Verify that CCCD was successfully written
			try:
				res = self.ble_conn.expect('Characteristic value was written successfully', timeout=10)
			except pexpect.TIMEOUT, e:
				print "ERROR: CCCD timeout"
				sys.exit(0)
				return False
		        
				#Print messages if VERBOSE mode is enabled
				#ble_msg_verbose()

    #--------------------------------------------------------------------------
    # Send the Init info (*.dat file contents) to peripheral device.
    #--------------------------------------------------------------------------
    def _dfu_send_init(self):
        debug_msg("dfu_send_info")

        # Open the DAT file and create array of its contents
        bin_array = array('B', open(self.datfile_path, 'rb').read())

        # Transmit Init info
        self._dfu_data_send_req(bin_array)

    #--------------------------------------------------------------------------
    # Initialize: 
    #    Hex: read and convert hexfile into bin_array 
    #    Bin: read binfile into bin_array
    #--------------------------------------------------------------------------
    def input_setup(self):

        print "Sending file " + self.hexfile_path + " to " + self.target_mac

        if self.hexfile_path == None:
            raise Exception("input invalid")

        name, extent = os.path.splitext(self.hexfile_path)

        if extent == ".bin":
            self.bin_array = array('B', open(self.hexfile_path, 'rb').read())
            self.hex_size = len(self.bin_array)
            print "bin array size: ", self.hex_size
            return

        if extent == ".hex":
            intelhex = IntelHex(self.hexfile_path)
            self.bin_array = intelhex.tobinarray()
            self.hex_size = len(self.bin_array)
            print "bin array size: ", self.hex_size
            return

        raise Exception("input invalid")
    
    def _dfu_check_mode(self):
        
        self._dfu_get_handles()
        print self.ctrlpt_cccd_handle
        print self.ctrlpt_handle
        print self.data_handle
        
        debug_msg("_dfu_check_mode")
        #look for DFU switch characteristic
		
        resetHandle = getHandle(self.ble_conn, UUID.DFU_Control_Point)  
        
        print "resetHandle " + str(resetHandle)
        
        self.ctrlpt_cccd_handle=None
        
        if not resetHandle:
            # maybe it already is IN DFU mode
            self.ctrlpt_handle = getHandle(self.ble_conn, UUID.DFU_Control_Point)
            if not self.ctrlpt_handle:
                print "Not in DFU, nor has the toggle characteristic, aborting.."
                return False
        
        if resetHandle or self.ctrlpt_handle:
            if resetHandle:
                print "Switching device into DFU mode"
                print 'char-write-cmd 0x%02s %02x' % (resetHandle, 1)
                self.ble_conn.sendline('char-write-cmd 0x%02s %02x' % (resetHandle, 1))
                time.sleep(0.2)
        
                print "Node is being restarted"
                self.ble_conn.sendline('exit')
                time.sleep(0.2)
                self.ble_conn.kill(0)
        
                # wait for restart
                time.sleep(5)
                print "Reconnecting..."
        
                # reinitialize
                #self.__init__(self.target_mac, self.hexfile_path, self.interface)
                self.__init__(self.target_mac, self.hexfile_path, self.datfile_path)
                #self.__init__(self.target_mac, self.hexfile_path)
                # reconnect
                connected = self.scan_and_connect()
                
                print "connected " + str(connected)
        
                if not connected:
                    return False
        
                return self._dfu_check_mode()
            else:
                print "Node is in DFU mode"
            return True
        else:
        
            return False

    def _dfu_get_handles(self):
        print "_dfu_get_handles"
        #s110
        #self.ctrlpt_cccd_handle = '0e'
        #self.data_handle = '0b'
        
        #s132
        self.ctrlpt_cccd_handle = '10'
        self.data_handle = '0e'
        
        
        ctrlpt_cccd_handle = getHandle(self.ble_conn,"00002902-0000-1000-8000-00805f9b34fb")
        data_handle = getHandle(self.ble_conn,"00001532-1212-efde-1523-785feabcd123")
        
        print "ctrlpt_cccd_handle " + str(ctrlpt_cccd_handle)
        print "data_handle " + str(data_handle)
        
        if ctrlpt_cccd_handle:
            self.ctrlpt_cccd_handle = ctrlpt_cccd_handle
        if data_handle:
            self.data_handle = data_handle

    def switch_in_dfu_mode(self):
		"""
		Enable CCD to switch in DFU mode
		"""
		debug_msg("switch_in_dfu_mode")

		#Enable notifications 
		#debug_msg(str('char-write-cmd 0x%02s %02x' % (self.ctrlpt_cccd_handle, 1)))  #char-write-req 0x0014 0100
		#self.ble_conn.sendline('char-write-req 0x%02s %02x' % (self.ctrlpt_cccd_handle, 1))

		if(self._dfu_enable_cccd(False)==False): #Try this
			return False
		time.sleep(0.5)

		#TODO handle softdevice and bootloader upgrade
		#print  "Send 'START DFU' + Application Command"
		#Reset the board in DFU mode. After reset the board will be disconnected
		debug_msg(str('char-write-req 0x%02x 0104' % (self.reset_handle))) #char-write-req 0x0013 0104
		self.ble_conn.sendline('char-write-req 0x%02x 0104' % (self.reset_handle))  #Reset
		self.ble_conn.sendline('') #BR
		try:
			res = self.ble_conn.expect('.* Invalid file descriptor', timeout=10)
		except pexpect.TIMEOUT, e:
			print "ERROR: Reset timeout"
			return False

		debug_msg("END switch_in_dfu_mode")
		#Reconnect the board.
		ret = self.scan_and_connect()
		print "Connected " + str(ret)
        
    def switch_in_dfu_mode_alt(self):
		debug_msg("switch_in_dfu_mode")
		# scan for characteristics:
		status = self._dfu_check_mode()
		print "status " + str(status)
		if not status:
			return False

    """
    --------------------------------------------------------------------------
     Send the binary firmware image to peripheral device.
    --------------------------------------------------------------------------
    """
    def dfu_send_image(self):
		debug_msg("dfu_send_image")

		if not self._check_DFU_mode():
			self.switch_in_dfu_mode()

		debug_msg("Enable Notifications in DFU mode")
		self._dfu_enable_cccd(True)

		#TODO Handle softdevice and bootloader upgrade
		# Send 'START DFU' + Application Command
		self._dfu_state_set(0x0104)

		# Transmit binary image size
		hex_size_array_lsb = convert_uint32_to_array(len(self.bin_array))

		#print hex_size_array_lsb
		self._dfu_data_send_req(hex_size_array_lsb)
		debug_msg("Sending hex file size")

		# Send 'INIT DFU' Command
		self._dfu_state_set(0x0200)

		# Wait for INIT DFU notification (indicates flash erase completed)
		notify = self._dfu_wait_for_notify()

		# Check the notify status.
		dfu_status = self._dfu_parse_notify(notify)
		if dfu_status != "OK":
			raise Exception("bad notification status")

		# Transmit the Init image (DAT).
		self._dfu_send_init()

		# Send 'INIT DFU' + Complete Command
		self._dfu_state_set(0x0201)

		# Send packet receipt notification interval (currently 10)
		self._dfu_pkt_rcpt_notif_req()

		# Send 'RECEIVE FIRMWARE IMAGE' command to set DFU in firmware receive state. 
		self._dfu_state_set_byte(Commands.RECEIVE_FIRMWARE_IMAGE)

		'''
		Send bin_array contents as as series of packets (burst mode).
		Each segment is pkt_payload_size bytes long.
		For every pkt_receipt_interval sends, wait for notification.
		'''
		segment_count = 1
		for i in range(0, self.hex_size, self.pkt_payload_size):

			segment = self.bin_array[i:i + self.pkt_payload_size]
			self._dfu_data_send_cmd(segment)

			#print "segment #", segment_count

			if (segment_count % self.pkt_receipt_interval) == 0:
				notify = self._dfu_wait_for_notify()

				if notify == None:
					raise Exception("no notification received")

				dfu_status = self._dfu_parse_notify(notify)

				if dfu_status == None or dfu_status != "OK":
					raise Exception("bad notification status")

			segment_count += 1

		# Send Validate Command
		self._dfu_state_set_byte(Commands.VALIDATE_FIRMWARE_IMAGE)

		# Wait a bit for copy on the peer to be finished
		time.sleep(1)

		# Send Activate and Reset Command
		self._dfu_state_set_byte(Commands.ACTIVATE_FIRMWARE_AND_RESET)
		
		"""
		--------------------------------------------------------------------------
			Return True if is already in DFU mode
		--------------------------------------------------------------------------
		"""
    def _check_DFU_mode(self):
		print "Checking DFU State..."
		res=False
                self.ble_conn.before = ""
                self.ble_conn.buffer = ""
		self.ble_conn.sendline('char-read-uuid %s' % UUID.DFU_Version)
		
		
		try:
                       res=self.ble_conn.expect('value: .*? \r\n')
				
		except pexpect.TIMEOUT, e:
			#print "[ERROR]_check_DFU_mode: State timeout"
			pass
		except:
			pass

		msg_ret = self.ble_conn.after
                debug_msg(msg_ret)

		if msg_ret.find("value: 08 00")!=-1:		
			res=True
			print "Board already in DFU mode"
		elif msg_ret.find("value: 01 00")!=-1:		
			print "Board needs to switch in DFU mode"
		else:
			print "[ERROR]: Invalid state"
			sys.exit(1)

		return res
		
    """
    --------------------------------------------------------------------------
     Disconnect from peer device if not done already and clean up. 
    --------------------------------------------------------------------------
    """
    def disconnect(self):
        self.ble_conn.sendline('exit')
        print "*****OK*****"
        self.ble_conn.close()
        


def getDFUControlPointHandle(ble_connection, uuid):
    ble_connection.before = ""
    ble_connection.sendline('char-desc')

    found = False
    while not found:
        try:
            ble_connection.expect([uuid], timeout = 3)

            #check if the given descriptor belongs to DFU_Control_Point characterist
            found = re.findall(UUID.DFU_Control_Point, ble_connection.before)
            if len(found) != 0 :
                found = True

        except pexpect.TIMEOUT, e:
            debug_msg("[ERROR] DFU service not found")
            sys.exit(1)

    handles = re.findall(r"handle: 0x..(..)", ble_connection.before)
    ble_connection.before = ""
    ble_connection.buffer = ""

    if len(handles) > 1:
        return handles[-1], handles[-2]
    else:
        return 0, 0

        
def getHandle(ble_connection, uuid):
    print "getHandle " + uuid
    in_characteristic = True
    ble_connection.before = ""
    ble_connection.sendline('characteristics')
    try:
        ble_connection.expect([uuid], timeout=2)
        handles = re.findall(r"char value handle: 0x..(..)", ble_connection.before)
        print handles
        ble_connection.before = ""
        ble_connection.buffer = ""
    except pexpect.TIMEOUT, e:
        in_characteristic = False

    if not in_characteristic:
        ble_connection.sendline('char-desc')
        try:
            ble_connection.expect([uuid], timeout=2)
            handles = re.findall(r"0x..(..)", ble_connection.before)
            print handles
            ble_connection.before = ""
            ble_connection.buffer = ""
        except pexpect.TIMEOUT, e:
            return False

    if len(handles) > 0:
        return handles[-1]
    else:
        return False

def main():
    """
    ------------------------------------------------------------------------------
    
    ------------------------------------------------------------------------------
    """

    print "DFU Server start"
    try:
        parser = optparse.OptionParser(usage='%prog -f <hex_file> -a <dfu_target_address>\n\nExample:\n\tdfu.py -f application.hex -d application.dat -a cd:e3:4a:47:1c:e4',
                                       version='0.5')

        parser.add_option('-a', '--address',
                  action='store',
                  dest="address",
                  type="string",
                  default=None,
                  help='DFU target address.'
                  )

        parser.add_option('-f', '--file',
                  action='store',
                  dest="hexfile",
                  type="string",
                  default=None,
                  help='hex file to be uploaded.'
                  )

        parser.add_option('-d', '--dat',
                  action='store',
                  dest="datfile",
                  type="string",
                  default=None,
                  help='dat file to be uploaded.'
                  )

        parser.add_option('-z', '--zip',
                  action='store',
                  dest="zipfile",
                  type="string",
                  default=None,
                  help='zip file to be used.'
                  )

        options, args = parser.parse_args()

    except Exception, e:
        print e
        print "For help use --help"
        sys.exit(2)

    try:

        ''' Validate input parameters '''

        if not options.address:
            parser.print_help()
            exit(2)

        unpacker = None
        hexfile  = None
        datfile  = None

        if options.zipfile != None:

            if (options.hexfile != None) or (options.datfile != None):
                print "Conflicting input directives"
                exit(2)

            unpacker = Unpacker()
            #print options.zipfile
            try:
            	hexfile, datfile = unpacker.unpack_zipfile(options.zipfile)	
            except Exception, e:        
                print "ERR"
                print e
                pass

        else:
            if (not options.hexfile) or (not options.datfile):
                parser.print_help()
                exit(2)

            if not os.path.isfile(options.hexfile):
                print "Error: Hex file doesn't exist"
                exit(2)

            if not os.path.isfile(options.datfile):
                print "Error: DAT file doesn't exist"
                exit(2)

            hexfile = options.hexfile
            datfile = options.datfile

        ''' Start of Device Firmware Update processing '''
        ble_dfu = BleDfuServer(options.address.upper(), hexfile, datfile)

        # Initialize inputs
        ble_dfu.input_setup()

		#Debug
        #ble_dfu.__init__(ble_dfu.target_mac, ble_dfu.hexfile_path, ble_dfu.datfile_path)
        # Connect to peer device.
        if ble_dfu.scan_and_connect():
            # Transmit the hex image to peer device.
            ble_dfu.dfu_send_image()
    
            # Wait to receive the disconnect event from peripheral device.
            time.sleep(1)
    
            # Disconnect from peer device if not done already and clean up. 
            ble_dfu.disconnect()

    except Exception, e:
        print e
        pass

    except:
        pass

    # If Unpacker for zipfile used then delete Unpacker
    #if unpacker != None:
    #    unpacker.delete()

    debug_msg("DFU Server done")

"""
------------------------------------------------------------------------------

------------------------------------------------------------------------------
"""
if __name__ == '__main__':

    # Do not litter the world with broken .pyc files.
    sys.dont_write_bytecode = True

    main()

