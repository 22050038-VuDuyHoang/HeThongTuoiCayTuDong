HỆ THÔNG TƯỚI CÂY TỰ ĐỘNG  
  
Thành viên:   Vũ Duy Hoàng  
                Nguyễn Hoàng Vũ  
                  
Mô tả hệ thống tưới cây tự động:
- Hệ thống tưới cây tự động sử dụng vi điều khiển ESP32 kết hợp với các cảm biến (độ ẩm đất, mưa, nhiệt độ & độ ẩm không khí DHT22, ánh sáng LDR, mực nước) để thu thập dữ liệu môi trường và quyết định bật/tắt bơm qua relay. Hệ thống hỗ trợ hai chế độ hoạt động:
- Tự động (Auto): dựa vào dữ liệu cảm biến và ngưỡng cài đặt để tự động tưới khi đất khô, dừng khi đủ ẩm, trời mưa hoặc bồn cạn nước.
- Thủ công (Manual): người dùng có thể điều khiển bơm trực tiếp bằng nút bấm hoặc từ xa thông qua Node-RED Dashboard.
- Dữ liệu cảm biến và trạng thái hệ thống được truyền qua MQTT, hiển thị real-time trên dashboard, giúp người dùng dễ dàng giám sát và điều khiển từ xa. Hệ thống có ưu điểm tiết kiệm nước, giảm công sức, chi phí thấp, dễ mở rộng, phù hợp hộ gia đình, trường học, hoặc mô hình nông nghiệp nhỏ.
