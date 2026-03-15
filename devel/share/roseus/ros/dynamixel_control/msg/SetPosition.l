;; Auto-generated. Do not edit!


(when (boundp 'dynamixel_control::SetPosition)
  (if (not (find-package "DYNAMIXEL_CONTROL"))
    (make-package "DYNAMIXEL_CONTROL"))
  (shadow 'SetPosition (find-package "DYNAMIXEL_CONTROL")))
(unless (find-package "DYNAMIXEL_CONTROL::SETPOSITION")
  (make-package "DYNAMIXEL_CONTROL::SETPOSITION"))

(in-package "ROS")
;;//! \htmlinclude SetPosition.msg.html


(defclass dynamixel_control::SetPosition
  :super ros::object
  :slots (_id _position ))

(defmethod dynamixel_control::SetPosition
  (:init
   (&key
    ((:id __id) 0)
    ((:position __position) 0)
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _position (round __position))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:position
   (&optional __position)
   (if __position (setq _position __position)) _position)
  (:serialization-length
   ()
   (+
    ;; int32 _id
    4
    ;; int32 _position
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _id
       (write-long _id s)
     ;; int32 _position
       (write-long _position s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _position
     (setq _position (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get dynamixel_control::SetPosition :md5sum-) "833110e1050ad82a9398ab64153a6a86")
(setf (get dynamixel_control::SetPosition :datatype-) "dynamixel_control/SetPosition")
(setf (get dynamixel_control::SetPosition :definition-)
      "int32 id
int32 position

")



(provide :dynamixel_control/SetPosition "833110e1050ad82a9398ab64153a6a86")


