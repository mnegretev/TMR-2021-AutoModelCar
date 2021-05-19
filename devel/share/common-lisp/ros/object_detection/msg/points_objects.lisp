; Auto-generated. Do not edit!


(cl:in-package object_detection-msg)


;//! \htmlinclude points_objects.msg.html

(cl:defclass <points_objects> (roslisp-msg-protocol:ros-message)
  ((points
    :reader points
    :initarg :points
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point)))
   (another_field
    :reader another_field
    :initarg :another_field
    :type cl:fixnum
    :initform 0))
)

(cl:defclass points_objects (<points_objects>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <points_objects>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'points_objects)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name object_detection-msg:<points_objects> is deprecated: use object_detection-msg:points_objects instead.")))

(cl:ensure-generic-function 'points-val :lambda-list '(m))
(cl:defmethod points-val ((m <points_objects>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_detection-msg:points-val is deprecated.  Use object_detection-msg:points instead.")
  (points m))

(cl:ensure-generic-function 'another_field-val :lambda-list '(m))
(cl:defmethod another_field-val ((m <points_objects>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_detection-msg:another_field-val is deprecated.  Use object_detection-msg:another_field instead.")
  (another_field m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <points_objects>) ostream)
  "Serializes a message object of type '<points_objects>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'points))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'another_field)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <points_objects>) istream)
  "Deserializes a message object of type '<points_objects>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'points) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'points)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'another_field)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<points_objects>)))
  "Returns string type for a message object of type '<points_objects>"
  "object_detection/points_objects")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'points_objects)))
  "Returns string type for a message object of type 'points_objects"
  "object_detection/points_objects")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<points_objects>)))
  "Returns md5sum for a message object of type '<points_objects>"
  "52b7cc5e44032cacb08a65980d276cc0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'points_objects)))
  "Returns md5sum for a message object of type 'points_objects"
  "52b7cc5e44032cacb08a65980d276cc0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<points_objects>)))
  "Returns full string definition for message of type '<points_objects>"
  (cl:format cl:nil "geometry_msgs/Point[] points~%uint8 another_field~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'points_objects)))
  "Returns full string definition for message of type 'points_objects"
  (cl:format cl:nil "geometry_msgs/Point[] points~%uint8 another_field~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <points_objects>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <points_objects>))
  "Converts a ROS message object to a list"
  (cl:list 'points_objects
    (cl:cons ':points (points msg))
    (cl:cons ':another_field (another_field msg))
))
