#include <linux/proc_fs.h>
#include <linux/sched.h> 
#include <linux/seq_file.h>
#include <linux/kallsyms.h>
#include <linux/utsname.h>
#include <asm/uaccess.h>

#define SEQ_printf(m, x...)	    \
 do {			    \
    if (m)		    \
	seq_printf(m, x);	\
    else		    \
	printk(x);	    \
 } while (0)


static DEFINE_MUTEX(my_apsbp_lock);

//by major, for apsbp
static unsigned int apsbp_id = 0xFFFFFFFF;



//by major, for apsbp
unsigned int mdbootreadsbp(void)
{
    unsigned int tmp_apsbp_id = 0;

    //printk("need to read apsbp_id=%d\n",apsbp_id);

    mutex_lock(&my_apsbp_lock);

    tmp_apsbp_id = apsbp_id;

    mutex_unlock(&my_apsbp_lock);
    
    return tmp_apsbp_id;
}

//by major, for apsbp
static int apsbp_show(struct seq_file *m, void *v)
{
    //printk("mdbootup, apsbp_show start");
    SEQ_printf(m, "%d\r\n",apsbp_id);
    //printk("mdbootup, apsbp_show end");	
    return 0;
}
static int apsbp_open(struct inode *inode, struct file *file) 
{ 
    return single_open(file, apsbp_show, inode->i_private); 
} 

static ssize_t apsbp_write(struct file *filp, const char *ubuf,
	   size_t cnt, loff_t *data)
{
    char buf[50] ={0};
    size_t copy_size = cnt;
    //printk("apsbp_write copy_size=%d\r\n",copy_size);
    if (cnt >= sizeof(buf))
	copy_size = sizeof(buf) - 1;

    if (copy_from_user(&buf, ubuf, copy_size))
	return -EFAULT;

    buf[copy_size] = 0;
    
    memcpy(&apsbp_id,buf, sizeof(apsbp_id));  
    //printk("apsbp_write apsbp_id=%d\r\n",apsbp_id);

    return cnt;

}
//by major, for apsbp  end


//by major, for apsbp config
static const struct file_operations apsbp_bootlog_fops = { 
    .open = apsbp_open, //thi is for cat /proc/apsbp 
    .write = apsbp_write,
    .read = seq_read, 
    .llseek = seq_lseek, 
    .release = single_release, 
};

static int __init init_apsbp_bootup(void)
{
    struct proc_dir_entry *pe;

    //by major, for apsbp config
    pe = proc_create("apsbp", 0666, NULL, &apsbp_bootlog_fops);
    if (!pe){
	//return -ENOMEM;
        printk("Bootup, error: apsbp create failed!");
    }

    return 0;
}
__initcall(init_apsbp_bootup);

