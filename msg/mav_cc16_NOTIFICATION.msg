# Automatically Generated in 2017-06-12 22:33:47.452630
# MESSAGE: NOTIFICATION
# Description:Sends a notification to the app
uint8 ID = 0
uint8 sysid
uint8 compid

Header header

uint8 type              # Type of the notification (see enum NOTIFICATION_TYPE)
char[50] description    # Notification
char[30] tag            # Tag

uint8 NOTIFICATION_TYPE_DEBUG = 0             # Debug Notification
uint8 NOTIFICATION_TYPE_INFO = 1              # Info Notification
uint8 NOTIFICATION_TYPE_WARNING = 2           # Warning Notification
uint8 NOTIFICATION_TYPE_CRITICAL = 3          # Critical Notification
