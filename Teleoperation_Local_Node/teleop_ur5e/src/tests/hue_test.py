import cv2
import mediapipe as mp
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands

window_title = 'Recognition result'
trackbar_title = 'Hue-offset'
trackbar_titleA = 'S'
trackbar_titleB = 'V-Brightness'

c = 0
b = 0
hue = 0

class HueHelper:
    def __init__(self):
        self.mp_hands = mp_hands.Hands(static_image_mode=True,
                                        max_num_hands=2,
                                        min_detection_confidence=0.5)
        self.img_bgr = None

    def apply_hue_offset(self,image):# 0 is no change; 0<=huechange<=180
        # convert img to hsv
        #image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        h = img_hsv[:,:,0]
        s = img_hsv[:,:,1]
        v = img_hsv[:,:,2]
        # shift the hue
        # cv2 will clip automatically to avoid color wrap-around
        img_h = cv2.add(h, hue)
        img_s = cv2.add(s, c)
        
        lim = 255 - b
        v[v > lim] = 255
        v[v <= lim] += b
        # combine new hue with s and v
        img_hsv = cv2.merge([img_h,img_s,v])
        #img_hsv = cv2.convertScaleAbs(img_hsv, beta=5)
        # convert from HSV to BGR
        return cv2.cvtColor(img_hsv, cv2.COLOR_HSV2BGR)

    def on_trackbar_change(self, trackbar_hue_offset):
        global hue
        hue = trackbar_hue_offset
        img_bgr_modified = self.recognize(self.img_bgr.copy())
        cv2.imshow(window_title, img_bgr_modified)

    def on_trackbar_changeA(self, trackbar_contrast):
        global c
        c = trackbar_contrast
        img_bgr_modified = self.recognize(self.img_bgr.copy())
        cv2.imshow(window_title, img_bgr_modified)

    def on_trackbar_changeB(self, trackbar_brightness):
        global b
        b = trackbar_brightness
        img_bgr_modified = self.recognize(self.img_bgr.copy())
        cv2.imshow(window_title, img_bgr_modified)

    def recognize(self, img_bgr):
        
        img_bgr = self.apply_hue_offset(img_bgr)
        print("H C B = ", hue, " ", c, " ", b)
        img_rgb = cv2.addWeighted( img_bgr, c, img_bgr, 0, b) #cv2.convertScaleAbs(img_bgr, alpha=c, beta=b)
        #img_rgb = cv2.convertScaleAbs(img_bgr, alpha=2, beta=10) #cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        results = self.mp_hands.process(img_rgb)

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                mp_drawing.draw_landmarks(
                    img_bgr,
                    hand_landmarks,
                    mp_hands.HAND_CONNECTIONS,
                    mp_drawing_styles.get_default_hand_landmarks_style(),
                    mp_drawing_styles.get_default_hand_connections_style())
        else:
            print('no hands were found')
        return img_bgr
    
    def run(self, img_path):
        #cap = cv2.VideoCapture(0)
        self.img_bgr = cv2.imread(img_path)
        self.img_bgr = cv2.resize(self.img_bgr, (720, 480))
        if self.img_bgr is None: print('Image was not found!')
        self.on_trackbar_change(hue)
        self.on_trackbar_changeA(c)
        self.on_trackbar_changeB(b)
        # Hue range is 0-179: https://docs.opencv.org/4.x/df/d9d/tutorial_py_colorspaces.html
        cv2.createTrackbar(trackbar_titleA, window_title, 0, 200, self.on_trackbar_changeA)
        cv2.createTrackbar(trackbar_titleB, window_title, 0, 200, self.on_trackbar_changeB)
        cv2.createTrackbar(trackbar_title, window_title, 0, 179, self.on_trackbar_change)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

if __name__ == '__main__':
    h = HueHelper()
    h.run("Hand Gesture Recognition_screenshot_11.05.2024.png")