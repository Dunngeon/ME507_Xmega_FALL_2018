//*************************************************************************************
/** @file encoderDriver.cpp
 *	  Provides encoderDriver class to read the encoders built onto our motors.
 *
 *  @b Revisions:
 *    11-28-18 RT Original file
 *
 *  @b Usage:
 *    This file is intended to be used on an XMEGA MCU with JGA25-371 gearmotors.
 *    Instances of this class should be generated separate from the corresponding
 *    motorDriver class and have its output sent to the motor class within a task.
 *
 *  @b License:
 *    This file is copyright 2018 by Ricky Tan and released under the Lesser GNU
 *    Public License, version 3. It intended for educational use only, but its use
 *    is not limited thereto, except by the fact that it's not really useful for
 *    anything else, so nobody in his right (or left) mind would try to use it.
 *    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *    ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *    LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUEN-
 *    TIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 *    OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */
//*************************************************************************************

// Header file for this project
#include "encoderDriver.h"
