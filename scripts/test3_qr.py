import pyzbar.pyzbar as pyzbar
import cv2

cap = cv2.VideoCapture(0) #cam ON

i = 0
img=cap.read()

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) #image colour convertion
     
decoded = pyzbar.decode(gray) #decode the scanned qr
barcode_type=0
barcode_data='0'


for d in decoded: 
  x, y, w, h = d.rect

  barcode_data = d.data.decode("utf-8")
  barcode_type = d.type

  cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)

  text = '%s (%s)' % (barcode_data, barcode_type)
  cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2, cv2.LINE_AA)
  
 
  
  #break

cv2.imshow('img', img)

  
  
 

dummy=[barcode_data]
  #print(dummy)
  #print(type(barcode_data))


bubble_gum = dummy[0].split(',')
unnaku_thevayana_value = map(float,bubble_gum)
print(unnaku_thevayana_value)



cap.release()
cv2.destroyAllWindows()
