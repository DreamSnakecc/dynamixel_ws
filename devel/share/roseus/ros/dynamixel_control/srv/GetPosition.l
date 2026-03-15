;; Auto-generated. Do not edit!


(when (boundp 'dynamixel_control::GetPosition)
  (if (not (find-package "DYNAMIXEL_CONTROL"))
    (make-package "DYNAMIXEL_CONTROL"))
  (shadow 'GetPosition (find-package "DYNAMIXEL_CONTROL")))
(unless (find-package "DYNAMIXEL_CONTROL::GETPOSITION")
  (make-package "DYNAMIXEL_CONTROL::GETPOSITION"))
(unless (find-package "DYNAMIXEL_CONTROL::GETPOSITIONREQUEST")
  (make-package "DYNAMIXEL_CONTROL::GETPOSITIONREQUEST"))
(unless (find-package "DYNAMIXEL_CONTROL::GETPOSITIONRESPONSE")
  (make-package "DYNAMIXEL_CONTROL::GETPOSITIONRESPONSE"))

(in-package "ROS")





(defclass dynamixel_control::GetPositionRequest
  :super ros::object
  :slots (_id ))

(defmethod dynamixel_control::GetPositionRequest
  (:init
   (&key
    ((:id __id) 0)
    )
   (send-super :init)
   (setq _id (round __id))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:serialization-length
   ()
   (+
    ;; int32 _id
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _id
       (write-long _id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass dynamixel_control::GetPositionResponse
  :super ros::object
  :slots (_position ))

(defmethod dynamixel_control::GetPositionResponse
  (:init
   (&key
    ((:position __position) 0)
    )
   (send-super :init)
   (setq _position (round __position))
   self)
  (:position
   (&optional __position)
   (if __position (setq _position __position)) _position)
  (:serialization-length
   ()
   (+
    ;; int32 _position
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _position
       (write-long _position s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _position
     (setq _position (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass dynamixel_control::GetPosition
  :super ros::object
  :slots ())

(setf (get dynamixel_control::GetPosition :md5sum-) "954ac3955681827bc0e9997911512d36")
(setf (get dynamixel_control::GetPosition :datatype-) "dynamixel_control/GetPosition")
(setf (get dynamixel_control::GetPosition :request) dynamixel_control::GetPositionRequest)
(setf (get dynamixel_control::GetPosition :response) dynamixel_control::GetPositionResponse)

(defmethod dynamixel_control::GetPositionRequest
  (:response () (instance dynamixel_control::GetPositionResponse :init)))

(setf (get dynamixel_control::GetPositionRequest :md5sum-) "954ac3955681827bc0e9997911512d36")
(setf (get dynamixel_control::GetPositionRequest :datatype-) "dynamixel_control/GetPositionRequest")
(setf (get dynamixel_control::GetPositionRequest :definition-)
      "int32 id
---
int32 position

")

(setf (get dynamixel_control::GetPositionResponse :md5sum-) "954ac3955681827bc0e9997911512d36")
(setf (get dynamixel_control::GetPositionResponse :datatype-) "dynamixel_control/GetPositionResponse")
(setf (get dynamixel_control::GetPositionResponse :definition-)
      "int32 id
---
int32 position

")



(provide :dynamixel_control/GetPosition "954ac3955681827bc0e9997911512d36")


