from pyzbar.pyzbar import decode
from pyzbar.pyzbar import ZBarSymbol
import cv2

barcodes = []

BLACK_THRESHOLD = 200
THIN_THRESHOLD = 200

resizefactors = [1, 2, 4, 6]

image = cv2.imread('pic2.jpg')
imageGRY = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
cv2.imwrite('gray.jpg', imageGRY)

ret, bw_im = cv2.threshold(imageGRY, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
#cv2.imwrite('bw_im.jpg',bw_im)

def detectQR(product_img, input_img, contour, factor):
    x, y, w, h = cv2.boundingRect(cnt)
    if factor == 1 and h < THIN_THRESHOLD or w < THIN_THRESHOLD:
        return False, product_img, None

    roi = input_img[y:y + h, x:x + w]
    if factor != 1:
        roi = cv2.resize(roi, (int(roi.shape[1]*factor), int(roi.shape[0]*factor)), cv2.INTER_NEAREST)

    b = decode(roi, symbols=[ZBarSymbol.QRCODE])
    for barcode in b:
        (x1, y1, w, h) = barcode.rect
        if factor != 1:
            x1 = int(x1/factor)
            y1 = int(y1/factor)
            w = int(w/factor)
            h = int(h/factor)

        barcodeData = barcode.data.decode("utf-8")
        barcodeType = barcode.type
        cv2.rectangle(product_img, (x+x1, y+y1), (x+x1 + w, y+y1 + h), (0, 0, 255), 4)
        # draw the barcode data and barcode type on the image
        text = "{} ({})".format(barcodeData, barcodeType)
        cv2.putText(product_img, text, (x+x1, y+y1 - 10), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 0, 255), 2)
        print("[INFO] Resize {}x, Found {} barcode: {}".format(f, barcodeType, barcodeData))

    return True, product_img, b


contours, hierarchy = cv2.findContours(bw_im, 1, 3)
for cnt in contours:
    for f in resizefactors:
        validdim, image, blist = detectQR(image, imageGRY, cnt, f)
        if validdim is False:
            break
        elif len(blist) != 0:
            for b in blist:
                barcodes.append(b.data.decode("utf-8"))
            break
        else:
            # Redundant
            continue

# Remove duplicate barcodes
barcodes = list(set(barcodes))
print("barcode count: %d" % (len(barcodes)))

image = cv2.resize(image, (int(image.shape[1]/2), int(image.shape[0]/2)))
cv2.imwrite('product.jpg',image)

