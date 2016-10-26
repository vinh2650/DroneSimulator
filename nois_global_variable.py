_samp = 0
_freq = 105.3e6
_fft_size = 1024
_count = 1

_Analog_sig_source = None
_Blocks_throttle = None
_Wxgui_fftsink = None
_uhd_usrp_source_0 = None
#set value for sample Rate
def setSampleRate(samp):
	global _samp
	_samp = samp
#get value for sample Rate
def getSampleRate():
	global _samp
	return  _samp
#set value for Frequency
def setFreq(freq):
        global _freq
        _freq = freq
#get value for Frequency
def getFreq():
        global _freq
        return  _freq

#set value for Analog_sig_source 
def setAnalog_sig_source (analog_sig_source):
        global _Analog_sig_source
        _Analog_sig_source  = analog_sig_source
        print 'SET ANALOG SIG SOURCE'
#get value for Analog_sig_source
def getAnalog_sig_source():
        print 'GET ANALOG SIG SOURCE'
        global _Analog_sig_source
        return  _Analog_sig_source
#set value for Wxgui_fftsink 
def setWxgui_fftsink (_wxgui_fftsink):
        global _Wxgui_fftsink
        _Wxgui_fftsink  = _wxgui_fftsink
        print 'SET _Wxgui_fftsink'
#get value for Analog_sig_source
def getWxgui_fftsink():
        print 'GET _Wxgui_fftsink SIG SOURCE'
        global _Wxgui_fftsink
        return  _Wxgui_fftsink

#set value for uhd_usrp_source_0 
def setuhd_usrp_source_0 (uhd_usrp_source_0):
        global _uhd_usrp_source_0
        _uhd_usrp_source_0  = uhd_usrp_source_0
        print 'SET _uhd_usrp_source_0'
#get value for Analog_sig_source
def getuhd_usrp_source_0():
        print 'GET _uhd_usrp_source_0'
        global _uhd_usrp_source_0
        return  _uhd_usrp_source_0



#set value for Blocks_throttle
def setBlocks_throttle(blocks_throttle):
        global _Blocks_throttle
        _Blocks_throttle  = blocks_throttle
#get value for Blocks_throttle
def getBlocks_throttle():
        global _Blocks_throttle
        return  _Blocks_throttle


