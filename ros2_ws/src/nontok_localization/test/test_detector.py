

def test_decode_frame_takes_a_jpeg_buffer_or_a_decoded_frame():
    import cv2
    import numpy as np
    from nontok_localization.camera import decode_frame
    image = np.zeros((48, 64, 3), np.uint8)
    image[:, 32:] = 255
    jpeg = cv2.imencode('.jpg', image)[1]
    for raw in (jpeg, jpeg.reshape(1, -1), jpeg.reshape(-1, 1)):       # drivers differ in the buffer's shape
        gray = decode_frame(raw, gray=True)
        assert gray.shape == (48, 64) and gray[10, 5] < 30 and gray[10, 60] > 225
    assert decode_frame(jpeg, gray=False).shape == (48, 64, 3)
    assert decode_frame(image, gray=True).shape == (48, 64)            # the driver decoded it anyway
    assert decode_frame(image, gray=False) is image
