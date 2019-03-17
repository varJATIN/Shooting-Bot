# Shooting-Bot
Automatic shooting gun (prototype for Indian defence army ).




    import numpy as np
    import cv2
    first_frame=None
    video=cv2.VideoCapture(0)
    a=0
    while True:
     a=a+1
     chk,img=video.read()
     gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
     gauss=cv2.GaussianBlur(img,(21,21),0)
    
    if first_frame is None:
        first_frame=gray
        continue
     delta_frame=cv2.absdiff(first_frame,gray) 
     ret,thresh_delta=cv2.threshold(delta_frame,30,255,cv2.THRESH_BINARY)
     img_dilate=cv2.dilate(thresh_delta,None, iterations=0)
     im2,cntr,hier=cv2.findContours(thresh_delta.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
     for contour in cntr:
         if cv2.contourArea(contour)<100:
             continue
         #x,y,w,h=cv2.boundingRect(contour)
         #cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
         m=cv2.moments(contour)
         cx=int(m["m10"]/m["m00"])
         cy=int(m["m01"]/m["m00"])
         cv2.circle(img,(cx,cy),9,(255,1,0),-1)
     cv2.imshow("Guassian blur",gauss)
     cv2.imshow("delta",delta_frame)
     cv2.imshow("detected",img)
     #cv2.imshow("thresh",thresh_delta)
     key=cv2.waitKey(1)
     if key==ord('q'):
       break
    print(a)
    video.release()
    cv2.destroyAllWindows()
