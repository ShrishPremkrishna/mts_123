import numpy as np
from scipy.signal import butter, sosfiltfilt, welch
from pyOpenBCI import OpenBCICyton
import time
import serial
import math

#boards
CYTON_PORT = '/dev/cu.usbserial-DP05IS0W'
ESP_PORT = '/dev/cu.usbserial-0001'

#conversion counts
uVoltsConv = (4500000)/24/(2**23-1) #uV/count
accelGConv = 0.002 / (2**4) #G/count

#sample manipulation
epochChannelData = np.zeros((2, 256))
samplesInEpoch = 0
sampleFreq = 250
samplesPerEpoch = 256

#exponential moving average
alphaEMA = 0.05
previous = 0
appliedEMA = 0

#calibration
baseline = 0
capacity = 0
baselineSamples = []
capacitySamples = []
calibrationEpochs = 0

#writing
lastSteering = 0
lastThrottle = 0
reverseTrue = -1

#handle deceleration
pastVals = [0, 0, 0]
lastDecelValue = 500
curPos = 0

allSamples = []

#define bandpass filter. we care about theta (4-8 hz) and beta (12-40 hz), so the optimal passband range would be 4-40 hz.
highPass = 4 
lowPass = 40

BANDPASS_SOS = butter(
  N=2, #research suggests that order 2 filter applied backwards and forwards (total order 4, eliminating phase shift) is most optimal for eeg analysis
  Wn=[highPass, lowPass],
  btype='bandpass',
  output='sos', #second-order sections are optimal for filtering because they break down higher-order filtering into numerically stable steps
  fs = sampleFreq
)

board = OpenBCICyton(port=CYTON_PORT, daisy=False, baud=115200, timeout=None, max_packets_skipped=1) #initialize boards
esp = serial.Serial(port=ESP_PORT, timeout=0.01)
time.sleep(2)

def calibrateRange(currentAttention):
  global calibrationEpochs
  global baseline, capacity, baselineSamples, capacitySamples
  global previous, appliedEMA

  if (calibrationEpochs == 0):
    input("press ENTER to start baseline calibration: ")
    calibrationEpochs += 1
    board.start_stream(throttle)
  elif (calibrationEpochs < 4): #skip first 3 unreliable epochs
    calibrationEpochs += 1
  elif (calibrationEpochs < 63):
    baselineSamples.append(currentAttention) #first minute - baseline
    print(f"{currentAttention}")
    calibrationEpochs += 1
  elif (calibrationEpochs == 63):
    board.stop_stream()
    input("press ENTER to start capacity calibration: ")
    calibrationEpochs += 1
    appliedEMA = 0
    board.start_stream(throttle)
  elif (calibrationEpochs < 67): #skip first 3 unreliable epochs
    calibrationEpochs += 1
  elif (calibrationEpochs < 118):
    capacitySamples.append(currentAttention) #1-2 minutes - capacity
    print(f"{currentAttention}")
    calibrationEpochs += 1
  elif (calibrationEpochs == 118):
    board.stop_stream()
    baseline = min(baselineSamples) #set baseline and capacity values. set initial previous value to baseline as a starting point (needed for EMA)
    previous = baseline
    capacity = max(capacitySamples)
    print(f"Baseline Measured: {baseline}")
    print(f"Capacity Measured: {capacity}")
    input("press ENTER to end calibration and begin racing")
    calibrationEpochs += 1
    board.start_stream(driveCar)

def handleSteering(sample):
  global lastSteering, lastThrottle
  if(sample.aux_data[0] == 0 and sample.aux_data[1] == 0 and sample.aux_data[2] == 0): #quit if no accelerometer data available
    return 

  accelX = sample.aux_data[0] * accelGConv #extract acceleration in x direction from sample packet
  accelX = max(-1.0, min(1.0, accelX)) #clamp values to range between -1 and 1 to pass arcsin function

  deg = math.asin(accelX) * (180 / math.pi) #get degree of tilt and convert from radians to degrees

  if (abs(deg) < 5): #create dead zone between -5 and 5 deg
    deg = 0
  elif (deg < -5): #based on car servo sensitivity
    deg = (deg + 5) * 2
  elif (deg > 5):
    deg = (deg - 5) * 2

  deg = max(-90, min(deg, 90))
  lastSteering = deg

  esp.write(f"S{lastSteering:03.0f}T{lastThrottle:03.0f}\n".encode()) #write degree of tilt to ESP with last throttle value

def throttle(sample):
  global allSamples
  global samplesInEpoch, epochChannelData, samplesPerEpoch
  global calibrationEpochs, baseline, capacity
  global lastSteering, lastThrottle, reverseTrue

  convertedSample = np.array(sample.channels_data[0:2]) * uVoltsConv #convert sample packet data from byte values to units of microvolts. we only care about PFC for attention, which is why we're only taking fp1 and fp2 readings

  allSamples.append(convertedSample) #for plotting

  #we need to segment into epochs to complete intensity vs time data, apply filters, and then extract features

  #epoch segmentation - for smooth throttle control, we need around 1 Hz updates at least, so roughly 1000 ms long epochs. Because our Cyton's 
  #sampling rate is 250 Hz, we get 1 sample every 4 ms. 1024 is a nice number near 1000 divisible by 4, so we can use 1024 ms as our epoch length.
  #this gives us 256 samples per epoch (1024 ms / 4 ms per 1 sample).
  if (samplesInEpoch == samplesPerEpoch): #if we've reached the end of an epoch
    epochChannelData = np.array(epochChannelData)

    filteredSignal = applyBandpass(epochChannelData)
    cleanedSignal = removeArtifacts(filteredSignal)

    attentionMetric = extractAttention(cleanedSignal)

    smoothed = applyEMA(attentionMetric)

    if (calibrationEpochs <= 118): #if currently in calibration, execute calibration code
      epochChannelData = np.zeros((2, samplesPerEpoch))
      samplesInEpoch = 0
      calibrateRange(smoothed)
      return

    smoothed = ((smoothed - baseline) / (capacity - baseline + 1e-10)) * 100 #fit smoothed bandpower values to our calibrated range, normalizing between 1 and 100
    smoothed = max(0, min(smoothed, 100)) #constrain to valid values, 1-100 normalized range
    if (smoothed < 10):
      smoothed = 0 #set a reasonable resting buffer
    else:
      smoothed -= 5 #correct rest of values down

    decelCorrected = upscaleDecel(smoothed)

    lastThrottle = decelCorrected * reverseTrue #correct for reverse state, handled by user button press

    epochChannelData = np.zeros((2, samplesPerEpoch))
    samplesInEpoch = 0
    esp.write(f"S{lastSteering:03.0f}T{lastThrottle:03.0f}\n".encode()) #write attention metric to ESP
    print(decelCorrected)
  
  epochChannelData[:, samplesInEpoch] = convertedSample
  samplesInEpoch += 1

def applyBandpass(signal):
  signal = np.asarray(signal) #convert to ndarray if needed
  filtered = sosfiltfilt(BANDPASS_SOS, signal, axis=1) #apply previously defined bandpass filter
  return filtered

def removeArtifacts(signal):
  return signal

def extractAttention(signal):
  global sampleFreq
  global samplesPerEpoch
  betaMin, betaMax = 12, 15

  #welch's method is a super powerful way of computing a power spectral density. the function applies an FFT to convert to the spectral domain,
  #then estimates PSD by dividing the data into overlapping segments and averaging the periodograms to yield a numerically stable and accurate result
  freqsExtracted, transformedPSD = welch(signal, fs=sampleFreq, window='hann', nperseg=samplesPerEpoch, noverlap=64, axis=1)

  #at this point, we have two important arrays post Fourier Transform:
    #transformedPSD is a spectral domain array that conveys PSD vs frequency bin. 
    #freqsExtracted gives us the Hz values for each frequency bin in transformed.

  betaFreqs = (freqsExtracted >= betaMin) & (freqsExtracted <= betaMax)

  df = freqsExtracted[1] - freqsExtracted[0] #compute unit changes in frequency in our transformedPSD array

  #split transformedPSD into 4 arrays, representing beta and theta range for each channel. integrate each over frequency to get total band power
  #because PSD is given in units of power per hertz. 
  betaCh1 = np.sum(transformedPSD[0, betaFreqs]) * df
  betaCh2 = np.sum(transformedPSD[1, betaFreqs]) * df

  betaBandPower = (betaCh1 + betaCh2) / 2

  attentionMetric = betaBandPower #final attention metric is raw beta bandpower, this is justified by research & field testing
  attentionMetric = max(0, min(100, attentionMetric))

  return attentionMetric 

#we need an exponential moving average to smooth our data --> elucidate general trend of bandpower values rather than getting jerky data
def applyEMA(currentBeta):
  global alphaEMA, previous, appliedEMA

  if (appliedEMA < 3): #if first data point, we don't have a previous to work with. we set it, and return the raw data. we also need to delay applying the EMA for the 3 initial skipped epochs.
    previous = currentBeta
    appliedEMA += 1
    return currentBeta
  
  #applying some empirically-determined scalar alpha to the next piece of data, then adding the product of the scalar's complement and the previous smoothed data
  smoothed = (alphaEMA * currentBeta) + ((1 - alphaEMA) * previous)
  previous = smoothed

  return smoothed

#natural neurological state tends towards activity/focus. therefore, the rate at which humans can unfocus is drastically slower than the rate at which we can focus, hence the need for this function.
def upscaleDecel(currentAttention):
  global pastVals, curPos, lastDecelValue
  result = currentAttention
  pastVals[curPos] = currentAttention #update our circular queue

  oldestPos = curPos + 1 #handle index updating for circular queue
  if (oldestPos > 2):
    oldestPos = 0
  prevPos = curPos - 1
  if (prevPos < 0):
    prevPos = 2

  if (pastVals[curPos] < pastVals[oldestPos] - 3): #exponentially decelerate car if attempt to relax is evident
    if (lastDecelValue == 500):
      result = result / 2
    else:
      result = lastDecelValue / 2
    lastDecelValue = result
  else: #gradually increase throttle back to normal if attempt to relax has vanished
    if (result > lastDecelValue):
      lastDecelValue *= 1.5
      result = lastDecelValue
    else: 
      lastDecelValue = 500

  curPos += 1 #handle index updating for circular queue
  if (curPos > 2):
    curPos = 0
  
  return result #return updated value

def handleButtons():
  global reverseTrue, calibrationEpochs

  btnPressed = esp.readline().decode().strip()  #read a line from serial
  if btnPressed:  #ignore empty reads
    if not btnPressed.startswith("EVT_BIN"): #EVT_BIN marks all button Serial inputs. any line that does not start with EVT_BIN is garbage to be ignored
      return
    
    if (btnPressed == "EVT_BTN1"): #button 1 is pin 25, which will be our reverse direction button
      print("direction toggled")
      reverseTrue *= -1
    elif (btnPressed == "EVT_BTN2"): #button 2 - pin 26, shut down
      print("emergency shutdown called")
      raise KeyboardInterrupt
    elif (btnPressed == "EVT_BTN3"): #button 3 - pin 27, fast forward calibration
      print("skip to next calibration state")
      if (calibrationEpochs < 63):
        calibrationEpochs = 63
      elif (calibrationEpochs < 118):
        calibrationEpochs = 118

#MAIN CALLBACK - CALL ALL FUNCTIONS
def driveCar(sample):
  handleSteering(sample)
  throttle(sample)
  handleButtons()

try:
  calibrateRange(0)

except KeyboardInterrupt:
  print("\nStopping stream...")
  esp.write(f"S000T000\n".encode()) #write neutral command to ESP
  board.stop_stream()
  esp.close()
  print("Stream stopped cleanly.")
