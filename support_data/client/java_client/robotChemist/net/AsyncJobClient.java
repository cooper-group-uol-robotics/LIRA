package robotChemist.net;

import java.io.IOException;
import java.net.InetSocketAddress;
import java.nio.ByteBuffer;
import java.nio.channels.SelectionKey;
import java.nio.channels.Selector;
import java.nio.channels.SocketChannel;
import java.util.Iterator;
import java.util.Set;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;


import com.kuka.task.ITaskLogger;

/***
 * This class allows asynchronous communication with an AsyncJobServer using sockets such that jobs messages 
 * can be exchanged between the arm and the base easily and with no blocking.
 * @author stoic-roboticist
 *
 */
public class AsyncJobClient {
	
	private SocketChannel client;
	private Selector selector;
	private ByteBuffer outBuffer;
	private ByteBuffer inBuffer;
	
	private LinkedBlockingQueue<JobMsg> inBoundMsgQueue;
	private JobMsg outMessage;
	private ExecutorService thread;
	
	private boolean running = false;
	private boolean connected = false;
	
	private JobEncoder encoder;
	private JobDecoder decoder;
	
	private ITaskLogger logger;
	
	/***
	 * Constructs the AsyncJobClient object using the AsyncJobServer TCP/IP information.
	 * @param remoteAddr - the AsyncJobServer TCP/IP address.
	 * @param port - the AsyncJobServer TCP/IP port.
	 * @param logger - RoboticsAPI logging interface.
	 * @throws IOException if any of the underlying {@link SocketChannel} methods fail.
	 */
	public AsyncJobClient(String remoteAddr, int port, ITaskLogger logger) throws IOException
	{
		this.selector = Selector.open();
		this.client = SocketChannel.open();
		client.configureBlocking(false);
		client.connect(new InetSocketAddress(remoteAddr, port));
		client.register(this.selector, SelectionKey.OP_CONNECT | SelectionKey.OP_READ);
		this.thread = Executors.newSingleThreadExecutor();
		this.inBuffer = ByteBuffer.allocate(256);
		this.outBuffer = ByteBuffer.allocate(256);
		inBoundMsgQueue = new LinkedBlockingQueue<JobMsg>(); 
		this.logger = logger;
		this.encoder = new JobEncoder();
		this.decoder = new JobDecoder();
	}
	
	/***
	 * Start communicating with the AsyncJobServer such that messages can be exchanged.
	 * This method launches a separate thread to handle the connection.
	 */
	public void start()
	{
		running = true;
		thread.execute(new Runnable() 
		{

			@Override
			public void run() {
				do
				{
					try 
					{
						if (selector.select() <= 0)
						{
							continue;
						}
						Set<SelectionKey> selectedKeys = selector.selectedKeys();
			            Iterator<SelectionKey> iter = selectedKeys.iterator();
			            while (iter.hasNext())
			            {
			            	SelectionKey key = iter.next();
			            	if (key.isConnectable())
			            	{
			            		establishConnection(key);
			            	}
			            	if (key.isReadable())
			            	{
			            		SocketChannel sc = (SocketChannel) key.channel();
			            		if (sc == client)
			            		{
			            			readMessageFromBuffer();
			            		}
			            	}
			            	iter.remove();
			            }
					}
					catch (IOException e)
					{
						logger.error(e.getMessage());
						running = false;
					}
					catch (InterruptedException e)
					{
						logger.error(e.getMessage());
						running = false;
					}
				}
				while(running && connected);
			}
		});
	}
	
	/***
	 * Terminates the connection with the AsyncJobServer and stops the connection thread.
	 * This method has to be called when terminating any application using the AsyncJobClient class. If the client
	 * is already terminated then this method does nothing.
	 * @throws InterruptedException
	 * @throws IOException
	 */
	public void stop() throws InterruptedException, IOException
	{
		running = false;
		connected = false;
		if (!thread.awaitTermination(5, TimeUnit.SECONDS))
		{
			thread.shutdownNow();
		}
		client.close();
	}
	
	private void establishConnection(SelectionKey key) throws IOException
	{
		logger.info("establishing connection");
		client = (SocketChannel) key.channel();
		while (client.isConnectionPending()) 
		{
			client.finishConnect();
         }
		connected = true;
		logger.info("connection complete");
	}
	
	private void readMessageFromBuffer() throws IOException, InterruptedException
	{
		synchronized(this)
		{
			inBuffer.clear();
			int inLength = client.read(inBuffer);
			inBuffer.flip();
			if (inLength > 0)
			{
				JobMsg inMessage = decoder.decode(inBuffer);
				inBoundMsgQueue.put(inMessage);
			}
		}
	}
	
	private void sendMessageFromBuffer() throws IOException
	{
		encoder.encode(outMessage, outBuffer);
		client.write(outBuffer);
		//System.out.println("sending " + outMessage);
		outMessage = null;
	}
	
	/***
	 * Sends the given {@link JobMsg} to the server asynchronously.
	 * @param job - the message to be sent.
	 * @throws IOException
	 */
	public synchronized void sendMessage(JobMsg job) throws IOException
	{
		
		outMessage = job;
		sendMessageFromBuffer();
	}
	
	/***
	 * Gets the messages queue received from the client.
	 * @return  the received message queue LinkedBlockingQueue<JobMsg>.
	 */
	public LinkedBlockingQueue<JobMsg> getMessageQueue()
	{
		return inBoundMsgQueue;
	}
	
	/***
	 * 
	 * @return true if connected to the AsyncJobServer
	 */
	public boolean isConnected()
	{
		return connected;
	}
}
