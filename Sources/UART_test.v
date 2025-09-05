module UART_test(
    input wire clock,
    input wire UART_RX,
    input wire reset,
    output wire UART_TX,
    output reg keyready,
    output reg uartip,
    output reg regisfull,
    output reg encryptionready,
    output reg decryptionready,
    output reg hashready
);

    // Wires for UART and display
    wire [7:0] Data;
    wire byte_received;
    reg [7:0] number = 0;
    
    reg [4:0] counter;
    reg [127:0] key128;
    
    

    // UART TX path
    reg [7:0] TX = 0;
    reg send = 0;
    wire busy;
    
    wire [127:0] aes_e128;   // "Encryption done"
    wire [127:0] aes_d128;   // "Decryption done"
    
    wire [255:0] hash_sha256; // "sha-256 Hash Ready"

    // UART Receiver module
    uart_reciever uart_rx_inst (
        .clock(clock),
        .RX_async(UART_RX),
        .Data(Data),
        .byte_recieved(byte_received)
    );

    // UART Transmitter module
    uart_transmitter uart_tx_inst (
        .clock(clock),
        .Data(TX),
        .send(send),
        .TX(UART_TX),
        .busy(busy)
    );
    
 AES aes_inst (
      .keyready (keyready),
      .key128   (key128),
      .e128     (aes_e128),
      .d128     (aes_d128),
      .e192     (/*unused*/),
      .d192     (/*unused*/),
      .e256     (/*unused*/),
      .d256     (/*unused*/)
      
    );

 sha256 sha_inst(.input_data (32'b00000000),
 .input_valid (1'b1),
 .input_ready (),
 .last_word (),
 .clk (clock),
 .rst (reset),
 .output_valid(),
 .hash_data(hash_sha256)
 );

    // Main behavior
    always @(posedge clock) begin
        send <= 0;  // Default to 0
        if (byte_received && !busy) begin
            number <= Data;   // Update display value
            TX <= Data;       // Echo the byte
            send <= 1;        // Trigger transmission
            uartip <= ~uartip; 
            key128 <= { key128[119:0], Data };
            counter = counter + 1;
            
            if(key128 != 128'b0)begin
                regisfull <= 1;
                hashready <= 1; end
            else  
                regisfull <= 0;   
                
            if(counter == 5'b10000) begin 
                keyready <= 1;
                counter <= 0;
                if (aes_e128)
                    encryptionready <= 1'b1;
                if (aes_d128)
                    decryptionready <= 1'b1;
            end       
            
            
        end
        else if (reset) begin
            key128 <= 128'b0;
            uartip <= 0;
            regisfull <= 0;
            keyready <= 0;
            counter <= 0;
            encryptionready <= 0;
            decryptionready <= 0;
            hashready <= 0;
            end
            
            
    end

endmodule