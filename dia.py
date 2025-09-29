#!/usr/bin/env python3
"""
Diagnostic script for speech recognition setup
Run this first to check if everything is working
"""

import sys
import subprocess

def check_package(package_name, import_name=None):
    """Check if a package is installed and importable"""
    if import_name is None:
        import_name = package_name.lower().replace('-', '_')
    
    try:
        # Try to import
        __import__(import_name)
        print(f"✅ {package_name}: OK")
        return True
    except ImportError:
        print(f"❌ {package_name}: NOT INSTALLED")
        return False
    except Exception as e:
        print(f"⚠️  {package_name}: ERROR - {e}")
        return False

def check_speech_recognition_details():
    """Check speech_recognition module in detail"""
    try:
        import speech_recognition as sr
        print(f"✅ SpeechRecognition version: {sr.__version__}")
        
        # Check if Recognizer class exists
        if hasattr(sr, 'Recognizer'):
            print("✅ sr.Recognizer class: Available")
            
            # Try to create recognizer
            try:
                recognizer = sr.Recognizer()
                print("✅ Recognizer creation: Success")
                return True
            except Exception as e:
                print(f"❌ Recognizer creation failed: {e}")
                return False
        else:
            print("❌ sr.Recognizer class: NOT FOUND")
            print("This suggests you have the wrong speech_recognition package!")
            return False
            
    except ImportError:
        print("❌ SpeechRecognition: Cannot import")
        return False

def check_audio():
    """Check PyAudio"""
    try:
        import pyaudio
        print(f"✅ PyAudio: OK")
        
        # Try to initialize
        p = pyaudio.PyAudio()
        device_count = p.get_device_count()
        print(f"✅ Audio devices found: {device_count}")
        
        # Look for SR-MV2000W
        found_sr_mv = False
        for i in range(device_count):
            try:
                device_info = p.get_device_info_by_index(i)
                if 'SR-MV2000W' in device_info.get('name', ''):
                    print(f"✅ SR-MV2000W found at index {i}")
                    found_sr_mv = True
                    break
            except:
                continue
        
        if not found_sr_mv:
            print("⚠️  SR-MV2000W not found (will work with default mic)")
        
        p.terminate()
        return True
        
    except ImportError:
        print("❌ PyAudio: NOT INSTALLED")
        return False
    except Exception as e:
        print(f"❌ PyAudio error: {e}")
        return False

def run_fix_commands():
    """Suggest and optionally run fix commands"""
    print("\n" + "="*50)
    print("SUGGESTED FIX COMMANDS:")
    print("="*50)
    
    commands = [
        "pip uninstall speech-recognition speechrecognition SpeechRecognition -y",
        "pip install SpeechRecognition",
        "pip install pyaudio"
    ]
    
    for cmd in commands:
        print(f"  {cmd}")
    
    print("\nWould you like to run these commands automatically? (y/n): ", end="")
    try:
        response = input().strip().lower()
        if response == 'y':
            for cmd in commands:
                print(f"\nRunning: {cmd}")
                try:
                    result = subprocess.run(cmd.split(), capture_output=True, text=True)
                    if result.returncode == 0:
                        print("✅ Success")
                    else:
                        print(f"❌ Failed: {result.stderr}")
                except Exception as e:
                    print(f"❌ Error running command: {e}")
            
            print("\nNow re-run this diagnostic script to check if it's fixed!")
        else:
            print("Please run the commands manually and then re-run this script.")
    except KeyboardInterrupt:
        print("\nSkipped automatic fix.")

def main():
    print("🔧 Speech Recognition Diagnostic Tool")
    print("="*50)
    print(f"Python version: {sys.version}")
    print(f"Platform: {sys.platform}")
    print("="*50)
    
    # Check basic packages
    packages_ok = True
    packages_ok &= check_package("pyaudio")
    packages_ok &= check_package("SpeechRecognition", "speech_recognition")
    
    print("\n" + "="*30)
    print("DETAILED SPEECH RECOGNITION CHECK:")
    print("="*30)
    sr_ok = check_speech_recognition_details()
    
    print("\n" + "="*20)
    print("AUDIO CHECK:")
    print("="*20)
    audio_ok = check_audio()
    
    print("\n" + "="*50)
    if packages_ok and sr_ok and audio_ok:
        print("🎉 ALL CHECKS PASSED! Your setup should work.")
        print("Try running the main script now:")
        print("  python sr_mv_speech_to_text.py")
    else:
        print("❌ ISSUES FOUND!")
        run_fix_commands()

if __name__ == "__main__":
    main()