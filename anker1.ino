#include "dw3000.h"
#define PIN_RST 27
#define PIN_IRQ 34
#define PIN_SS 4
#define RNG_DELAY_MS 1000
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385
#define ALL_MSG_COMMON_LEN 10
#define ALL_MSG_SN_IDX 2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN 4
#define POLL_TX_TO_RESP_RX_DLY_UUS 240
#define RESP_RX_TIMEOUT_UUS 400  //※１


/* Default communication configuration. We use default non-STS DW mode. 
デフォルトの通信設定。デフォルトの非STS DWモードを使用する。*/
static dwt_config_t config = {
    4,                /* Channel number. */
    DWT_PLEN_128,     /* Preamble length. Used in TX only. */
    DWT_PAC8,         /* Preamble acquisition chunk size. Used in RX only. */
    9,                /* TX preamble code. Used in TX only. */
    9,                /* RX preamble code. Used in RX only. */
    1,                /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,       /* Data rate. */
    DWT_PHRMODE_STD,  /* PHY header mode. */
    DWT_PHRRATE_STD,  /* PHY header rate. */
    (129 + 8 - 8),    /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. SFDタイムアウト（プリアンブル長＋1＋SFD長-PACサイズ）。RX でのみ使用。 */
    DWT_STS_MODE_OFF, /* STS disabled */
    DWT_STS_LEN_64,   /* STS length see allowed values in Enum dwt_sts_lengths_e */
    DWT_PDOA_M0       /* PDOA mode off */
};

static uint8_t tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0}; //これは送信するポーリングメッセージの内容を定義しています。
                                                                                            //0x41, 0x88: IEEE 802.15.4フレームのフレームコントロールフィールド
                                                                                            //0: シーケンス番号（後でframe_seq_nbの値で上書きされます）
                                                                                            //0xCA, 0xDE: PAN ID（Personal Area Network Identifier）
                                                                                            //'W', 'A', 'V', 'E': ASCII文字でメッセージ識別子
                                                                                            //0xE0: メッセージタイプ
                                                                                            //0, 0: 予備のバイト
static uint8_t rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};//これはアンカー（受信機）から期待される応答メッセージの形式を定義しています。
                                                                                                                    //構造はtx_poll_msgと似ていますが、識別子が'V', 'E', 'W', 'A'の順になっており、メッセージタイプが0xE1です。
                                                                                                                    // 末尾の0が多いのは、タイムスタンプなどの追加情報のためのスペースです。
static uint8_t frame_seq_nb = 0; //フレームシーケンス番号を保持する変数です
                                 // 各送信ごとにインクリメントされ、0から255の範囲で循環します。
static uint8_t rx_buffer[20]; //受信したデータを格納するためのバッファです。
                              //サイズは20バイトで、応答メッセージを格納するのに十分な大きさです。
static uint32_t status_reg = 0; //DW3000チップのステータスレジスタの値を保持するための変数です。
                                //チップの状態やイベントを追跡するのに使用されます。
static double tof; //電波の飛行時間（Time of Flight）を格納する変数です。
                   //距離計算に使用されます。
static double distance; //計算された距離を格納する変数です。
extern dwt_txconfig_t txconfig_options; //送信設定オプションを含む構造体への外部参照です。
                                        //この構造体は別のファイルで定義されており、送信パワーやパルス繰り返し周波数などの設定を含んでいます。


void setup()//
{
  UART_init(); //シリアル通信を初期化する

  spiBegin(PIN_IRQ, PIN_RST); // SPI通信を開始し、IRQピンとリセットピンを設定します。
  spiSelect(PIN_SS); //SPI通信で使用するスレーブセレクト（SS）ピンを設定するための関数呼び出し

  delay(2); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)

  while (!dwt_checkidlerc()) // Need to make sure DW IC is in IDLE_RC before proceeding
  {
    UART_puts("IDLE FAILED\r\n");
    while (1);
  }

  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
  {
    UART_puts("INIT FAILED\r\n");
    while (1);
  }

  // Enabling LEDs here for debug so that for each TX the D1 LED will flash on DW3000 red eval-shield boards.
  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

  /* Configure DW IC. See NOTE 6 below. */
   if (dwt_configure(&config)) /*if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device
   もし dwt_configure が DWT_ERROR を返したら、PLL か RX のキャリブレーションに失敗したのであろう。*/
   {
   UART_puts("CONFIG FAILED\r\n");
    while (1);
  }

  /* Configure the TX spectrum parameters (power, PG delay and PG count)  TXスペクトラム・パラメータ（パワー、PGディレイ、PGカウント）の設定*/
  dwt_configuretxrf(&txconfig_options);

  /* Apply default antenna delay value. See NOTE 2 below. */
  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);

  /* Set expected response's delay and timeout. See NOTE 1 and 5 below.
   * As this example only handles one incoming frame with always the same delay and timeout, those values can be set here once for all. */
  dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
  dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);//受信タイムアウト時間を設定　※１　受信モードに入った後、指定された時間（この場合400μs）だけ応答を待ちます。この時間内に有効なフレームを受信しなかった場合、タイムアウトイベントが発生します。

  /* Next can enable TX/RX states output on GPIOs 5 and 6 to help debug, and also TX/RX LEDs
   * Note, in real low power applications the LEDs should not be used. */
  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

  Serial.println("Range RX");
  Serial.println("Setup over........");
}



void loop()
{
  /* Write frame data to DW IC and prepare transmission. See NOTE 7 below. 
  DW ICにフレームデータを書き込み、送信準備を行う。下記NOTE7参照。*/
  tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb; //現在のフレームシーケンス番号を保持する変数です。
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK); //DW3000チップのレジスタ(記憶素子(少量高速ストレージ))に32ビット値を書き込みます。
  dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0); //送信データをDW3000チップのバッファ（一時格納）に書き込みます。　/* Zero offset in TX buffer. TX バッファのオフセットがゼロ。*/
  dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1);//送信フレーム制御パラメータを設定  /* Zero offset in TX buffer, ranging. TXバッファのゼロオフセット。*/

//送信開始
  /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay set by dwt_setrxaftertxdelay() has elapsed. 
  フレームが送信され、dwt_setrxaftertxdelay()で設定された遅延が経過した後、受信が自動的に有効になるように、応答が期待されることを示す送信を開始する。*/
  dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED); //即座に送信を開始します。送信完了後、自動的に受信モードに切り替わります。設定された時間内に応答を待ちます。

  /* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 8 below. 
  フレームの受信またはエラー/タイムアウトのポーリングが正しく達成されたと仮定する。以下の NOTE 8 を参照のこと。*/
  while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))  //DW3000チップのシステムステータスレジスタを読み取る
  {  //DW3000チップが応答を受信するか、タイムアウトまたはエラーが発生するまで待機
  };  

  /* Increment frame sequence number after transmission of the poll message (modulo 256).
  ポーリング・メッセージの送信後にフレーム・シーケンス番号をインクリメントする（256モジュロ）。 */
  frame_seq_nb++; // フレームシーケンス番号は、送信されるメッセージを一意に識別するために使用されます。各送信ごとに番号を増やすことで、新しいメッセージを前のメッセージと区別できます。


  if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) //UWB通信における重要なステップで、正常に受信したフレームのみを処理することで、通信の信頼性を確保しています。
  {
    uint32_t frame_len; //受信したフレーム（メッセージ）の長さを格納する変数

    /* Clear good RX frame event in the DW IC status register. */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK); //受信フレーム完了フラグをクリアして、次のフレーム受信に備えてチップの状態をリセットする　 DW3000チップの32ビットレジスタに値を書き込む  第1引数:書き込むレジスタのID  第2引数:書き込む32ビットのデータ

    /* A frame has been received, read it into the local buffer.  
    フレームが受信されたので、それをローカル・バッファに読み込む。*/
    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;  //受信したフレームの長さを取得して、rame_len変数にはフレームの実際の長さ（バイト数）が格納。 dwt_read32bitreg(RX_FINFO_ID)=DW3000チップの受信フレーム情報レジスタ（RX_FINFO）から32ビットの値を読み取ります。
    if (frame_len <= sizeof(rx_buffer)) //受信データがバッファに収まる場合のみ、データを読み取るようにします。受信したフレームの長さがバッファのサイズを超えていないかを確認する。　予期せぬ長さのフレームで悪意ある攻撃から守る
    {
      dwt_readrxdata(rx_buffer, frame_len, 0); //DW3000から受信したデータをrx_bufferに読み込みます。

      /* Check that the frame is the expected response from the companion "SS TWR responder" example.
      そのフレームが、「SS TWR レスポンダ」の例から期待される応答であることを確認する。
       * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame.
       フレームのシーケンス番号フィールドは関連性がないため、フレームの検証を簡素化するためにクリアされる。 */
      rx_buffer[ALL_MSG_SN_IDX] = 0; //受信したメッセージのシーケンス番号をクリア（0に設定） シーケンス番号を0にすることで、後続の比較処理を単純化している
      // ↑ ※おそらく、次のif文の判断をしやすくするためにデータを胆汁化している　UWB通信プロトコルにおいて、受信したメッセージが期待されるフォーマットに従っているかを確認する際の一般的なテクニックです。シーケンス番号を無視することで、メッセージの他の重要な部分（例：メッセージタイプ、送信元IDなど）に焦点を当てた検証が可能になります。

     //受信したメッセージ（rx_buffer）が期待される応答メッセージ（rx_resp_msg）と一致するかどうかを確認。　この条件が真の場合、メッセージは有効であると判断され、後続の処理（例：距離計算）が行われます。    偽の場合、メッセージは無視され、次の送受信サイクルに進みます。
      if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)
      {
        //タイムスタンプ変数:
        uint32_t poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts; //（※電波の往復時間を計算するため）ポーリングメッセージの送信時刻、応答メッセージの受信時刻、 ポーリングメッセージの受信時刻、応答メッセージの送信時刻
        //往復時間（Round Trip Delay）変数
        int32_t rtd_init, rtd_resp; //イニシエーター(動的IPアドレス側or接続要求を出す側)側での往復時間、 レスポンダー(固定IPアドレス側or接続要求を受ける側)側での往復時間
        //クロックオフセット比率:
        float clockOffsetRatio; //二つのデバイス間のクロック周波数の差を補正するための比率　　クロックオフセット比率を使用することで、デバイス間のクロックの微小な差異による誤差を補正し、より正確な距離測定が可能になります。

        /* Retrieve poll transmission and response reception timestamps. See NOTE 9 below.
        ポーリング送信および応答受信のタイムスタンプを取得する。以下の NOTE 9 を参照のこと。 */
        //ローカル（固定？）デバイスのタイムスタンプ読み取り・・・ポーリング送信時刻、応答受信時刻
        poll_tx_ts = dwt_readtxtimestamplo32(); //ポーリングメッセージの送信タイムスタンプを取得(読み取り＆代入？）
        resp_rx_ts = dwt_readrxtimestamplo32(); //応答メッセージの受信タイムスタンプを取得（読み取り＆代入？）

        /* Read carrier integrator value and calculate clock offset ratio. See NOTE 11 below. 
        キャリア積分器の値を読み取り、クロックオフセット比を計算する。下記のNOTE 11を参照してください。計算で必要？*/
        clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26); //DW3000チップの内部クロックオフセットを読み取り、それを比率として計算している。　clockOffsetRatioは、後続の距離計算で使用され、クロックの差異による誤差を補正します。

        /* Get timestamps embedded in response message. 
        応答メッセージに埋め込まれたタイムスタンプを取得します。受信したレスポンスメッセージから重要なタイムスタンプ情報を抽出しています。*/
        //リモート（動き）デバイスのタイムスタンプ読み取り・・・ポーリング受信時刻、応答送信時刻
        resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts); //
        resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

        /* Compute time of flight and distance, using clock offset ratio to correct for differing local and remote clock rates 
        クロックオフセット比を使用して、ローカルとリモートのクロックレートの違いを補正し、飛行時間と距離を”計算する”。*/

      //  if(...)//byte check
      //  {
        rtd_init = resp_rx_ts - poll_tx_ts;  //イニシエーター（距離測定を開始するデバイス）側での往復時間を計算 = (poll_tx_ts: ポーリングメッセージを送信した時刻) ー (resp_rx_ts: 応答メッセージを受信した時刻)
        rtd_resp = resp_tx_ts - poll_rx_ts;  //レスポンダー（応答するデバイス）側での往復時間を計算 = (ポーリングメッセージを受信した時刻) ー (resp_tx_ts: 応答メッセージを送信した時刻)

        tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;  //距離計算（電波の飛行時間（Time of Flight, ToF））
        distance = tof * SPEED_OF_LIGHT;  //距離 = 時間 × 速度(299,792,458)  distance・・・変数には、2つのデバイス間の実際の距離がメートル単位で格納されます。

        /* Display computed distance on LCD.
        　計算された距離をLCDに表示する。 */
          snprintf(dist_str, sizeof(dist_str), "DIST01: %3.2f m", distance);
          test_run_info((unsigned char *)dist_str);
       // }

      }
    }
  }
  else
  {
    /* Clear RX error/timeout events in the DW IC status register. 
    DW IC ステータスレジスタの RX エラー/タイムアウトイベントをクリアする。*/
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
  }

  /* Execute a delay between ranging exchanges. 
  レンジング交換の間にディレイを実行する。*/
 //Sleep(RNG_DELAY_MS);
 //delay(1);
}