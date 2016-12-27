/*
*  Add this file for operation proc device node
*/

#include <linux/proc_fs.h>
#include <linux/module.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>

#define char_length 30
static char global_buffer[char_length];

static int voice_call_nb_wb_flag_open(struct inode *inode, struct file *f)
{
	pr_err("%s\n", __func__);

	return 0;
}

static int voice_call_nb_wb_flag_release(struct inode *inode, struct file *f)
{
	pr_err("%s\n", __func__);

	return 0;
}

#if 0
static ssize_t voice_call_nb_wb_flag_read(struct file *file, char __user *buf,
                                                    size_t count, loff_t *offset)
{
    int len;
	int ret;

	pr_err("%s: enter \n", __func__);

	if(count>char_length-1)
		len = char_length-1;
	else
		len = count;	

    global_buffer[len] = '\0';
 	ret = copy_to_user((char *)buf, global_buffer, len+1);
	if(ret)
		return -1; 
	else{
      	pr_err("%s: buf = %s \n", __func__, buf);
		*offset += len+1;
        return len+1;
	}
}
#else
static ssize_t voice_call_nb_wb_flag_read(struct file *file, char __user *buf,
				    size_t len, loff_t *offset)
{
    ssize_t count = 0;   

    pr_err("%s: enter \n", __func__);
    if (*offset > 0) {
        count = 0;
    } else {
        pr_err("%s: buf = %s \n", __func__, global_buffer);
        count = sprintf(buf, "%s\n", global_buffer);
        *offset += count;
    }
    return count;
}
#endif

static ssize_t voice_call_nb_wb_flag_write(struct file *file, const char __user *buf,
				                                     size_t count, loff_t *offs)
{
    int len;
    int ret;  
 
    pr_err("%s: enter \n", __func__);

	memset(global_buffer, 0, char_length);
	if(count>char_length-1)
		len = char_length-1;
	else
		len = count;

	ret = copy_from_user(global_buffer, buf, len);
	if(ret)
		return -1;
    pr_err("%s: global_buffer = %s \n", __func__, global_buffer);

    return len;

}

static const struct file_operations my_node_fops = {
	.owner =    THIS_MODULE,
	.open =     voice_call_nb_wb_flag_open,
	.release =  voice_call_nb_wb_flag_release,
	.write =    voice_call_nb_wb_flag_write,
	.read = 	voice_call_nb_wb_flag_read,
};

static int __init proc_my_node_init(void)
{
	struct proc_dir_entry *proc_my_node;	
	proc_my_node = proc_create("my_node", S_IRWXUGO, NULL, &my_node_fops);

	if (!proc_my_node) 
	{
		printk(KERN_ERR"[lvrg]: unable to register '/proc/my_node'\n");
		return -ENOMEM;
	}
	return 0;
}

static void __exit proc_my_node_exit(void)
{
	remove_proc_entry("my_node", NULL);
}

module_init(proc_my_node_init);
module_exit(proc_my_node_exit);

MODULE_LICENSE("GPL v2");
